# H0 V12R17 + probe di capacità — verdetto: il tetto è l'inizializzazione

Data: 2026-08-17 (notte)

## Esito della strategia A+C autorizzata

**(A) Probe di capacità** (diagnostico, zero scritture canoniche, corpus P2
reale da 11.404 righe, schedule congelato):

| Fit | RMSE globale | Worst per-traiettoria | RMSE traiettorie seed-127 |
|---|---:|---:|---:|
| W256 H0-init (candidato reale) | 0,00498 | **0,00684** (seed 127) | ~0,0068 |
| W256 **fresh** | 0,00470 | 0,00567 | **0,0035 / 0,0041** |
| W512 fresh | 0,00476 | 0,00578 | 0,0035 / 0,0041 |

Conclusioni: **la capacità non è la leva** (W512 ≈ W256, falsificato a costo
zero); **la leva è l'inizializzazione** — l'ancora `same_byte_locked_h0`
tiene la rete in un bacino che fitta male proprio le righe di bordo; fresh
le fitta a metà errore. Artefatti con hash in `v12r17/diagnostics/`.

**(C) V12R17** (matrice P3 a 5 casi, seed 127 dichiarato `KNOWN_OPEN`):

- P0 PASS, P1 PASS 18/18 (riproduzione deterministica bit-exact della
  catena R16), P2 PASS;
- P3: `+0.20` **PASS riconfermato** (riproduzione del primo PASS storico);
  **`deterministic_offset_minus_0p20` FAIL terminale a step 71** —
  `grf_penetration` 25,557 mm al primo caricamento, 0 cicli, zero clipping.
  Nominale, 126 e 128 puri mai aperti.
  `FAIL_H0_V12R17_P3_PURE_PIPELINE_TERMINAL`.

## Perché questo verdetto chiude la strada della de-scope

Il `−0.20` **è uno dei tre start di training** del regime 15/07 (1,757 s):
non può essere escluso dalla qualifica senza rompere la copertura della
distribuzione su cui PPO campiona — l'argomento che rendeva difendibile la
de-scope del seed 127 (fuori distribuzione) qui non esiste. La matrice non
si può restringere oltre; **deve cambiare il candidato**.

## Stato di autonomia pura del candidato H0-init (dopo R16+R17)

| Caso | Puro | Sotto shield (P1) |
|---|---|---|
| `+0.20` | **PASS ×2** (500/500, 2 cicli, ~24,9 mm) | PASS a tutti gli alpha |
| `−0.20` | **FAIL @71** (25,557 mm) | PASS a tutti gli alpha (latch ~50%, picchi ~25 mm) |
| nominale | mai testato | PASS |
| seed 126/128 | mai testati | PASS |
| seed 127 | FAIL @202 (known-open) | FAIL @202 da 0,50 in su |

Lettura: la stabilità closed-loop è per-bacino e dipende dall'inizializzazione,
non monotona nella "difficoltà" storica del caso — il candidato ha imparato
il killer storico e resta fragile su uno start che sotto blend sembrava
domato. Coerente con il probe: il bacino H0-init ha una coda di errore sui
bordi che il fit non riesce a chiudere.

## Proposta per la decisione (mattina)

**V12R18 fresh-init W256, matrice piena a 6 casi**:

- stessa pipeline P0→P1→P2→P3 e stessi gate, con una sola modifica
  dichiarata: inizializzazione fresh (seed congelato) al posto del
  transplant H0 — supportata dai numeri del probe (coda dimezzata,
  seed-127 a 0,0035–0,0041);
- il vincolo di provenienza `same_byte_locked_h0` viene sostituito da
  "teacher provenance": l'imitazione resta interamente ancorata ai tape di
  H0 (le label non cambiano), cade solo l'ancora dei PESI iniziali;
- gate offline: da rivalidare col dry-fit deterministico prima del freeze
  (i numeri del probe SONO già il dry-fit: soglie invariate bastano con
  margine);
- onestà sul rischio: RMSE offline migliore non garantisce la fisica — ma è
  l'unica leva rimasta con evidenza a favore, e il probe ha falsificato
  l'alternativa (capacità) a costo zero.

In caso di FAIL fisico anche di R18: le opzioni residue diventano
strutturali (teacher oltre il tape congelato, o revisione del limite dei
25 mm a livello di sistema) e vanno discusse a mente fredda.

## File e artefatti

- `v12r17/` completo (freeze/lock/run root, P3 con forense del −0,20);
- `v12r17/diagnostics/`: probe di capacità (script `de450ac0…`, risultati
  `a0f120cc…`).

## Test e verifiche

- suite v12r17: 84/84 PASS pre-freeze; ruff PASS;
- catena P0→P2 riprodotta bit-exact (determinismo confermato per la terza
  volta);
- esiti letti da receipt/gate/summary/ledger reali.

## TODO

- [ ] Decisione: autorizzare V12R18 fresh-init W256 a matrice piena.
- [ ] In caso di PASS P3: port filiera Q3 → checkpoint-zero → morphology →
  preflight → training.
- [ ] Daily 2026-08-16 da consolidare al prossimo `end_day` (14 user report
  della giornata).
