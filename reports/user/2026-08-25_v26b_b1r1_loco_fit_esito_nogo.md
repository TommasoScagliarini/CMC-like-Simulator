# V26B B1R1 — fit LOCO a 6 fold: **NO-GO offline**, ma il difetto è cambiato natura

**Data**: 2026-08-25
**Esito**: preflight GO · fit eseguito · **verdetto NO-GO su 5 gate di 9**
**Nessun rollout, nessuna collection, nessuna fase Markov, nessuna promozione, nessun iperparametro corretto post-hoc.**
**Nessuna modifica a produzione, FSM v3, morfologia, contatto, reward, guardie, C++ o SEA.**

Il NO-GO di B1 (`e7328ac2…`) e B0 (`aa7ea0fa…`) sono verificati byte-identici. Nessuna directory di rollout B1R1 esiste.

---

## 1. Il risultato in una riga

**Tutti e sei i fold superano il gate di generalizzazione** (heldout ≤ 0,05): 0,0466 · 0,0265 · 0,0401 · 0,0259 · 0,0442 · 0,0233. Contro B1, dove due fold su tre fallivano e il peggiore era 0,1764.

Ciò che resta a fallire non è più la generalizzazione ma la **ricostruzione**, e si concentra quasi interamente nelle righe che il LOCO per costruzione non valida mai.

## 2. La regola di selezione, applicata come pinnata

Sei traiettorie di fit indipendenti, tutte partite **byte-identiche da B0** `b2429a82…`. Ogni fold ha registrato la MSE di validation a ogni epoca 1..400 senza arresto interno. La curva pooled è la media pesata per righe di validation. Applicata la regola di luglio a quella curva:

- **e\* = 358**, primo minimo della pooled (0,00126770)
- la pooled a 400 risale a 0,00143495: **il minimo è interno**, non un artefatto del budget
- `stopped_at_epoch = null`: il contatore stale ha raggiunto 42 delle 60 epoche prima che il budget finisse. Il minimo è genuino, ma la finestra di patience **non è stata esaurita** — lo segnalo per precisione, senza proporre di estendere il budget
- pooled a 100 / 200 / 300: 0,003141 / 0,001964 / 0,001867

Ogni fold è stato valutato **allo stesso e\* = 358**, mai al proprio best. Il fit finale è ripartito da B0 su tutte le 1500 righe per esattamente 358 epoche, senza validation.

Il test verifica che ri-eseguire un fold per k epoche riproduca **bit-per-bit** lo stato che il run da 400 aveva all'epoca k, quindi materializzare le curve e applicare la regola a posteriori non ha cambiato e\*.

## 3. Gate

| gate | soglia | osservato | esito |
|---|---|---|---|
| invarianti di integrità | — | tutte vere | **PASS** |
| equivalenza funzionale 25D | bit-exact | bit-identica | **PASS** |
| ogni fold migliora su B0 | — | 6/6 | **PASS** |
| **RMSE heldout per fold** | ≤ 0,05 | 0,0466 · 0,0265 · 0,0401 · 0,0259 · 0,0442 · 0,0233 | **PASS** |
| RMSE pooled di validation | ≤ 0,03 | **0,03560** | **FAIL** |
| RMSE finale aggregata | ≤ 0,02 | **0,03433** | **FAIL** |
| RMSE finale per giunto | ≤ 0,03 | knee **0,04048** · ankle 0,02681 | **FAIL** |
| max_abs finale per giunto | ≤ 0,15 | knee **0,17049** · ankle 0,13845 | **FAIL** |
| **ricostruzione WAIT** (non generalizzazione) | 0,02 / 0,03 / 0,15 | agg **0,02679** · knee rmse **0,03266** · ankle 0,01922 · max_abs 0,12686 / 0,09251 | **FAIL** |

Invarianti tutte verificate: 10 chiavi, larghezza 35, colonne 25:34 e clock 0:1 esattamente zero, nessun aggiornamento su di esse, logstd byte-identica a B0, critic mai caricato, equivalenza 25D bit-identica dopo il fit.

## 4. Dove sta l'errore — diagnosi

| sottoinsieme | righe | RMSE agg | knee rmse / max | ankle rmse / max |
|---|---|---|---|---|
| tutte | 1500 | 0,03433 | 0,04048 / 0,17049 | 0,02681 / 0,13845 |
| **coperte da heldout** | 910 | **0,02723** | 0,02982 / 0,09588 | 0,02436 / 0,07795 |
| **mai validate** | 590 | **0,04305** | 0,05286 / 0,17049 | 0,03020 / 0,13845 |
| — di cui WAIT | 187 | 0,02679 | 0,03266 / 0,12686 | 0,01922 / 0,09251 |
| — di cui **cicli parziali** | 403 | **0,04878** | 0,05997 / 0,17049 | 0,03411 / 0,13845 |

Le righe che i fold validano stanno a 0,02723; quelle mai validate a 0,04305, e dentro queste i **cicli parziali di testa e coda** sono a 0,04878, quasi il doppio.

**Tutte e 11 le righe con errore di ginocchio superiore a 0,15 sono nel gruppo mai validato.** La peggiore (0,17049) è plus020 locale 115, dentro il ciclo parziale iniziale.

Quindi i gate finali falliscono in larga parte per righe che il disegno LOCO, come approvato, non mette mai in validation: startup, coda e cicli incompleti. Non è un difetto del fit né della regola di selezione; è la conseguenza strutturale di validare solo cicli completi mentre i gate finali si misurano su tutte le 1500 righe.

## 5. Confronto con B1

| | B1 (LOTO 3) | B1R1 (LOCO 6) |
|---|---|---|
| fold heldout peggiore | 0,1764 | **0,0466** |
| fold che superano ≤0,05 | 1 su 3 | **6 su 6** |
| pooled / media LOTO | 0,09160 | **0,03560** |
| finale aggregata | 0,04128 | **0,03433** |
| finale knee rmse | 0,04910 | **0,04048** |
| finale ankle rmse | 0,03159 | **0,02681** |
| finale knee max_abs | 0,16001 | 0,17049 |

La correzione dello split ha fatto esattamente ciò per cui era stata progettata: la misura di generalizzazione è passata da 1/3 a 6/6, e il pooled è migliorato di 2,6×. Il max_abs del ginocchio è l'unica metrica peggiorata, e cade su una riga mai validata.

## 6. File

| file | SHA-256 |
|---|---|
| `v26b_b1r1_exec.py` | `e4d7260704754bf3b7f078942aa27177975bdbcc3bf77600bb657f534c13f9f7` |
| `test_v26b_b1r1_exec.py` | `40d0c7df0c4f81fbfa0685369aa530dbc20207cbbce79e88fa1a25a1cb1e907b` |
| `candidates/B1R1_BASE35_LOCO/v26b_b1r1_receipt.json` | `6a604ada500f6f51594500f596ba7a7797cd1350a110b9d0fe1807ee4d2cadbb` |
| `rl_module/module_state.pkl` | `9ffcdceb7f90f12cbcca0c151f8b667e3003ba4f48a124e630582d76cf0bd980` |
| `rl_module/actor_feature_manifest.json` | `c3f526534d340207da28e39c8a06fdabbc8034030dbc628bce296c392d977340` |
| log `b1r1_loco_fit_20260825_153530.log` | `8be6e5d7af65de22491019fd755d58771d76351540159f28c1bcf5ee331a4f85` |

Il receipt contiene le sei curve complete a 400 epoche, la curva pooled, la selezione, i sei fold valutati a e\*, la storia del fit finale e la ricostruzione WAIT.

**Comandi eseguiti**
```
python test_v26b_b1r1_exec.py         -> PASS, 66 check
python test_v26b_b1r1_loco_study.py   -> PASS, 87 check
python test_v26b_b0b1_masked35.py     -> PASS, 85 check
python v26b_b1r1_exec.py --preflight  -> GO, blockers []
python v26b_b1r1_exec.py --authorized-stage V26B-B1R1-BASE-FIT-LOCO
```

`fit_masked` è riusata **invariata** da `v26b_b_exec`: il test verifica che l'executor non la ridefinisca e che la funzione porti ancora `restore_logstd`, `project_columns`, `assert_projected` e i tre pesi di loss. Parent verificato solo B0: un percorso contenente `B1_BASE35_MASKED` o qualunque attore diagnostico è rifiutato per nome, e un digest sbagliato è rifiutato.

`git status` conferma produzione, FSM v3, contatto, reward, guardie e `tools/` intatti.

## 7. Cosa non ho fatto

Non ho toccato alcun iperparametro, non ho rilassato alcuna soglia, non ho rieseguito nulla dopo aver visto i numeri, e non ho lanciato rollout o collection. Il NO-GO è registrato come tale.

## 8. Decisioni che spettano a te

1. Se il NO-GO chiuda la fase base, oppure se i gate finali debbano essere misurati sulle righe che i fold validano davvero, dato che il 74% dell'eccesso di errore vive nelle 590 righe mai validate. È un cambiamento di definizione dei gate, non un rilassamento delle soglie, ma resta una tua decisione e non la applico.
2. Se il gate di ricostruzione WAIT a 0,02/0,03 sia raggiungibile: osservato 0,02679 aggregata e 0,03266 su knee, cioè mancato di poco su entrambe.
3. Se il budget di 400 epoche vada considerato adeguato: il minimo pooled è interno a 358 e la curva risale, ma la finestra di patience si è fermata a 42 stale su 60.

## 9. TODO propagati

- **TODO-2** — σ non assunto, da misurare prima delle recovery. *(aperto)*
- **TODO-3** — B3 indeterminato. *(aperto)*
- **TODO-4** — Conflitto cammino/plantarflessione. *(aperto)*
- **TODO-5** — Tensione interpolazione↔quota. *(aperto)*
- **TODO-7** — Penetrazione a 0,22 mm dalla guardia dura in REV4E. *(aperto)*
- **TODO-9** — Swing al 13,5% di non eseguibili. *(aperto)*
- **TODO-10** — Proxy autoregressivo rimosso per costruzione. *(indirizzato)*
- **TODO-14** — 590 righe mai validate sotto LOCO: **quantificato**, contengono l'errore dominante (0,04305 contro 0,02723) e tutte le 11 righe oltre 0,15 di errore sul ginocchio. *(aggiornato)*
- **TODO-15** — Fold 0 perde 20 righe WAIT dal training per l'embargo: accettato dall'architetto, il fit finale usa tutte e 187. *(chiuso per decisione)*
- **TODO-16** — Nuovo: la finestra di patience della curva pooled si è fermata a 42 stale su 60 per esaurimento del budget. *(nuovo)*
