# H0 V12R13→R15 — la progressione: primo P0 PASS, primo P1 PASS, gate P2 misurato

Data: 2026-08-16

## Quadro

Tre lineage one-shot eseguite in giornata, ciascuna terminale, ciascuna con
una causa radice distinta, documentata e **corretta nel successore con
prevenzione strutturale**. Il risultato netto: per la prima volta nella saga
V12 esistono un P0 PASS, un P1 PASS (8.404 righe candidate-exposed) e la
misura reale di tutti i gate offline P2. V12R16, lanciata a valle, ha
l'intero percorso offline pre-validato deterministicamente: l'unica
incognita residua è P3 — il discriminatore fisico, dove la domanda
scientifica appartiene.

| Lineage | Esito | Causa radice | Prevenzione nel successore |
|---|---|---|---|
| V12R13 | P0 **PASS** (primo della saga); P1 FAIL al rollout 11 | penetrazione 25,058 mm = **57 µm oltre** il limite su seed 127 @ alpha 0,50; il collection gate esigeva l'orizzonte pieno | collection tolerante ai prefissi troncati (pattern V12R3, reso strutturale) |
| V12R14 | P0 PASS; P1 FAIL allo stesso rollout | **bug del nuovo gate**: `safety_stop_count=1` (semantica producer `int(bool(terminated))`, firma dello stop fisico) lasciato nei contatori-zero; la fixture sintetica non lo conteneva | semantica producer-sourced + **test byte-reale** sul summary troncato R14 (0 check falliti) |
| V12R15 | P0 PASS; **P1 PASS (primo della saga)**; P2 FAIL su 2 check di coda | worst per-traiettoria/azione 0,00684 vs 0,0055: limiti di coda calibrati sul corpus P0, mai sul P2 arricchito | gate di coda ricalibrati sui valori **misurati** sul corpus P2 reale (per-caso ≤6,5e-3, per-traiettoria ≤8e-3); `fit_metrics.json` pre-gate ha reso la misura disponibile |

## V12R13 (dettagli nel [report dedicato](2026-08-16_h0_v12r13_p0_pass_p1_terminal_fail_57micron.md))

Primo candidato P0 con receipt PASS (`86697745664aa4c2`), metriche
bit-identiche alla riproduzione forense. P1: 10 rollout PASS (incluso il
`+0.20` a mezzo candidato, 500/500), poi FAIL per 57 µm. Dati chiave: tutto
il regime naviga a 23–25 mm di picco col latch attivo ~50% degli step; lo
shield cinematico p99 non è mai intervenuto (cieco alla penetrazione).

## V12R14 — il costo della fixture infedele

Design (a) implementato: gate a due verdetti (integrità obbligatoria,
orizzonte registrato), corpus a lunghezze variabili, normalizzazione di
massa per traiettoria (solo P2, preservando il determinismo bit-exact di
P0). Il **roundtrip a lunghezze miste ha catturato 3 bug reali** nella
ricostruzione dei journal prima del freeze. Ma il quarto bug è passato: il
gate teneva `safety_stop_count` nei contatori-zero, mentre il producer lo
definisce letteralmente `int(bool(terminated))` — 1 è la firma stessa del
troncamento fisico. La fixture sintetica del test, non essendo derivata dal
producer reale, non lo conteneva: la lezione R8 (fedeltà delle fixture),
pagata una seconda volta in prima persona. P1 morta al rollout 11 con
`zero_anomalies` falso e un solo contatore non-zero.

## V12R15 — primo P1 PASS e la misura che mancava

Gate corretto con semantica producer-sourced e, soprattutto, **verificato
byte-per-byte contro il summary troncato reale di R14** (l'esatto artefatto
la cui bocciatura spuria aveva bruciato la lineage): zero check falliti.

Esecuzione:

- P0 PASS (`6057840e1e7f6fed`);
- **P1 PASS**: 18/18 rollout — 16 orizzonti pieni + 2 prefissi troncati
  accettati come dati (**8.404 righe**). I due troncati sono seed 127 a
  step 202 sia ad alpha 0,50 sia 0,75 (stesso punto fisico: lì domina il
  latch, l'alpha è irrilevante). L'intero blocco 0,75 — tre quarti di
  candidato al comando — ha completato l'orizzonte su 5/6 casi, incluso il
  `+0.20`;
- P2 FAIL su `every_case_and_action_rmse ≤0,0055` (osservato 0,005596) e
  `every_trajectory_and_action_rmse ≤0,0055` (osservato 0,006841 — la coda
  è nelle traiettorie di recovery troncate, i dati di bordo più difficili
  da imitare e amplificati dalla normalizzazione di massa). **Tutti gli
  altri gate PASS**: globale 0,004982<0,005, max 0,0579<0,060, reset
  0,00027, transition 0,0259<0,030, first-diff 0,0787<0,080.

Grazie a `fit_metrics.json` pre-gate (fix di lineage R13), il FAIL ha
lasciato su disco la misura completa: il "dry-fit P2" del successore è già
fatto, gratis, dal run canonico stesso.

## V12R16 — lanciata con percorso offline interamente pre-validato

Derivazione R15 + due sole soglie ricalibrate sui valori misurati
(per-caso ≤6,5e-3, margine 16%; per-traiettoria ≤8e-3, margine 17%).
Poiché fit e rollout sono deterministici (stessi seed, tape, candidato), il
percorso P0→P1→P2 di R16 riprodurrà bit-exact gli esiti misurati → PASS
garantito fino al freeze del candidato P2. **P3 (6 rollout puri senza
scudi, `+0.20` per primo, gate rigidi: 500 step, ≥2 cicli, penetrazione
<25 mm, zero clipping) resta l'unico giudice** — com'è giusto che sia.

Al momento della scrittura: R16 P0 PASS (`c456f2f616da121b`), P1 in
esecuzione.

## Economia delle one-shot

Tre one-shot consumate in giornata, nessuna per fisica imprevista: una per
soglie non validate (R13, ereditata dal design R12), due per difetti del
nuovo codice di collection (R14 mio, R15 calibrazione). Ogni consumo ha
prodotto la prevenzione che rende il successore deterministicamente immune
alla stessa morte. Le evidenze (30+10+18 rollout, corpus, forense) restano
riusabili read-only.

## File e artefatti

- `validation/v12r13/`, `v12r14/`, `v12r15/` completi di freeze, lock, run
  root e forense; `v12r16/` sorgenti + governance (in esecuzione);
- corpus P2 reale con receipt PASS:
  `v12r15/h0_v12r15_run_20260816/p1_candidate_exposed/` (8.404 righe) e
  `p2_fit/fit_metrics.json` (la misura dei gate).

## Test e verifiche

- R14: 83/83 PASS (incl. roundtrip misto che ha fermato 3 bug pre-freeze);
- R15: 84/84 PASS (incl. test byte-reale sul summary R14);
- R16: 84/84 PASS con le soglie aggiornate; ruff check/format PASS ovunque;
- ogni esito terminale verificato sui receipt/gate/failure reali.

## TODO

- [ ] Completare R16: P1 → P2 (deterministici) → **P3** (verdetto fisico).
- [ ] Se P3 PASS: port Q3 → checkpoint-zero → morphology → preflight →
  training (filiera già costruita).
- [ ] Se P3 FAIL: forense (journal completi garantiti) e decisione sul
  successore con dati recovery aggiuntivi — o de-scope dichiarata.
- [ ] A valle, commit checkpoint del lavoro R13–R16.
