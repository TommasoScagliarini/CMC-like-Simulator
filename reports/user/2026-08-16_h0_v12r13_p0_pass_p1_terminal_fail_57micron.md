# H0 V12R13 — P0 PASS (primo della saga), P1 terminal FAIL per 57 micron

Data: 2026-08-16

## Esito

La lineage V12R13 (design V12R12 + gate offline ricalibrati dry-fit-validati
+ persistenza metriche pre-gate) è stata costruita, verificata (82/82 test,
ruff PASS), congelata ed eseguita one-shot:

- **P0 fit: PASS** — primo candidato con receipt PASS dell'intera saga V12.
  Candidate ID `AB06_H0_V12R13_V26_INVARIANT_SAFE_TEACHER:86697745664aa4c2`;
  metriche **bit-identiche** alla riproduzione forense che aveva calibrato i
  gate (RMSE globale `0,004530701113916344`, max `0,05541`, reset
  `0,000222`): il determinismo del fit ha funzionato esattamente come
  progettato, e `fit_metrics.json` è stato persistito prima del gate;
- **P1: terminal FAIL al rollout 11/18** —
  `alpha_0p50__stochastic_nominal_seed_127` terminato a **202/500 step** per
  `grf_penetration` con massimo **0,0250576682 m**: **57 micron oltre** il
  limite stretto `<0,025 m`. Il collection gate richiede l'orizzonte pieno
  → `FAIL_H0_V12R13_P1_CANDIDATE_EXPOSED`, pipeline chiusa senza retry.

Governance: freeze `c3218dba…`, lock `b800638f…` (fitter); freeze e lock P1
pubblicati e verificati. P2 e P3 mai aperti.

## I dati che contano

10 rollout completati su 11 tentati (tutti i 6 ad alpha 0,25 + 4 ad alpha
0,50, **incluso il `+0.20` a mezzo candidato, 500/500 step**):

| Rollout (alpha__caso) | Step | Pen. max | Latch attivo (step) |
|---|---:|---:|---:|
| 0,25 × 6 casi | 500 | 23–25 mm | 245–274 |
| 0,50 minus / nominale / plus / seed 126 | 500 | 23–25 mm | 246–274 |
| **0,50 seed 127** | **202** | **25,058 mm** | 100 |

Letture chiave:

1. **Tutti i rollout vivono sul filo**: penetrazione di picco a 23–25 mm
   ovunque, col safety latch che serve teacher puro per ~50% degli step. Il
   margine fisico dell'intero regime è ~1 mm (il tape sicuro stesso picca a
   24,3 mm). Il caso fallito non è qualitativamente diverso: ha sforato di
   0,06 mm dove gli altri restavano 0,1–2 mm sotto.
2. **Il supporto p99 non è mai intervenuto (0 trigger in 5.202 step)**: lo
   shield cinematico 18D non vede arrivare la penetrazione — il rischio vive
   in una dimensione (profondità di contatto) che l'envelope di supporto non
   osserva. Solo il latch causale sulla penetrazione lavora davvero.
3. Seed 127 si conferma il caso stocastico più duro della storia del
   progetto (già worst-case in V12R5 e V12R9).
4. Il `+0.20` — il killer storico — ha completato **due volte** l'orizzonte
   pieno (alpha 0,25 e 0,50) sotto shield: il masking a 18 feature +
   teacher da tape regge dove tutti i predecessori morivano, purché il
   latch possa intervenire.

## Perché è un avanzamento nonostante il FAIL

V12R13 è arrivata più avanti di qualunque lineage precedente: primo P0 PASS,
10 collection PASS, il caso critico storico domato sotto shield, e un FAIL
finale che non è un bug né un collasso ma un'escursione di 57 micron su un
limite che l'intero sistema sfiora per costruzione. Le 10 collection valide
(5.000 righe candidate-exposed etichettate dal tape) sono evidenza
riutilizzabile read-only dal successore, insieme al prefisso forense di 202
step del caso fallito.

## Opzioni per il successore (decisione richiesta)

Il rollout fallito è deterministico (noise congelato dal tape): rieseguirlo
identico riprodurrebbe lo stesso esito. Il successore deve cambiare una di
queste cose:

- **(a) Collection tolerante ai prefissi troncati** — il pattern storico
  V12R3→R9 (`recoverable_for_data_collection`): un rollout shielded
  terminato fisicamente resta dato valido di collection (mai PASS fisico).
  Richiede il refactor del layer corpus a lunghezze variabili
  (contabilità e ordering check per-traiettoria); è la soluzione onesta e
  robusta: nessun rollout può più bruciare la pipeline di raccolta.
- **(b) Latch più precoce** (ingresso 15→12 mm, rilascio 10→8 mm, costanti
  locali del successore senza toccare il modulo v10s congelato): con
  l'attivazione a 12 mm il caso 127 molto plausibilmente resta sotto 25 mm.
  Contro: il latch è già attivo ~50% degli step; anticiparlo diluisce
  ulteriormente l'esposizione del candidato (meno valore dei dati P1).
- **(c) = (a) + (b)** — cintura e bretelle: latch anticipato per completare
  più orizzonti, tolleranza ai prefissi perché nessun caso possa più
  costare una one-shot.

Raccomandazione: **(a)**, eventualmente (c) se si accetta la diluizione.
La (b) da sola lascia la pipeline esposta al prossimo caso da 57 micron.

## File e artefatti

- namespace `Trajectory Generator/baseline_MLP/validation/v12r13/`
  (10 sorgenti + test, freeze/lock fitter e P1);
- run root `h0_v12r13_run_20260816/`: `p0_fit/` completo (corpus,
  fit_metrics, gate, summary, receipt, candidato a 5 file),
  `p1_candidate_exposed/` con 10 collection PASS + forense del caso 11 +
  `failure.json` terminale;
- diagnostica pre-esistente in `v12r13/diagnostics/`.

Nessun namespace storico, plugin, tape o checkpoint toccato.

## Test e verifiche

- suite v12r13 pre-freeze: 82/82 PASS; ruff check/format PASS;
- preflight P0 PASS; freeze/lock verificati alla pubblicazione;
- P0: gate PASS con metriche identiche alla previsione deterministica;
- P1: 10 receipt di collection validi; failure.json e forense del caso 11
  completi (journal per-step, trace, summary, gate).

## TODO

- [ ] Autorizzare la lineage successore (V12R14) con l'opzione scelta
  (raccomandata: collection tolerante ai prefissi, importando le 10
  collection PASS e il prefisso forense di R13).
- [ ] In V12R14 valutare anche l'estensione dell'envelope di supporto con la
  dimensione di penetrazione (lo shield cinematico è cieco al rischio
  reale — dato nuovo di questa run).
- [ ] Dopo un P1 completo: P2 → P3 (il discriminatore fisico resta il punto
  aperto) → Q3 → checkpoint-zero → morphology → training.
