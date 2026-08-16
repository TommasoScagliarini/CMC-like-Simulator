# H0 V12R12 — P0 fit terminal FAIL sui gate offline e forense

Data: 2026-08-16

## Esito

La sequenza one-shot V12R12 è stata autorizzata ed eseguita fino al primo
stage. Cronologia:

- preflight P0 read-only: `PASS_H0_V12R12_P0_MASKED_TEACHER_PREFLIGHT`;
- protocol freeze pubblicato: SHA-256
  `9bc35aad610aa6cda971859fb2f3cb850fc0fbc1d22384bfe1a5c50439048274`;
- execution lock pubblicato: SHA-256
  `0fc031ca269efac77b2e9f38cf9af312cf7386d339de751138244fd1b33e51c5`;
- **`--execute --stage p0`: FAIL terminale**
  (`FAIL_H0_V12R12_MASKED_SAFE_TEACHER_FIT_TERMINAL`,
  `single fixed fit failed strict offline metric/state gates`).

Il gate ha bocciato **7 check metrici su 7**. P1, P2 e P3 non si sono mai
aperti; il run root `h0_v12r12_run_20260815/p0_fit/` contiene `corpus.npz`,
`gate.json` e `failure.json`. Nessun retry è autorizzato: **V12R12 è
terminale al fit P0**. Nessun rollout fisico è stato eseguito; detector,
GRF, SEA e artefatti storici sono intatti.

## Problema forense e riproduzione

`gate.json` persiste solo i booleani: i valori osservati non erano su disco
(il raise precede la scrittura del summary — stesso gap del `fit_p2` V12R3).
Poiché il fit è interamente deterministico (seed 20260815, full-batch,
`use_deterministic_algorithms`, schedule fisso), è stato riprodotto **in
memoria, senza alcuna scrittura canonica**, importando il fitter congelato e
chiamandone le sole funzioni pure. Artefatti della riproduzione:

- `Trajectory Generator/baseline_MLP/validation/v12r13/diagnostics/reproduce_v12r12_p0_fit_metrics.py`
  — SHA-256 `1b8e62d4d7a8c1b5724e46008989258761527fac1e82b2d644634dff37eb0026`;
- `…/v12r13/diagnostics/results/v12r12_p0_fit_reproduced_metrics.json`
  — SHA-256 `4803ecda806f96e7706ffd1c77b91eab0fa3723ec1772f5eae9858dc0c507ea2`.

## Metriche recuperate

| Metrica | Osservata | Gate V12R12 | Rapporto | Gate storico famiglia | Esito storico |
|---|---:|---:|---:|---:|---|
| RMSE globale | `0,004531` | `2,5e-4` | 18× oltre | `0,006` | sarebbe PASS |
| Max abs globale | `0,05541` | `3e-3` | 18× oltre | `0,060` | sarebbe PASS |
| Reset max abs | `0,000222` | `1e-5` | 22× oltre | `0,003` | sarebbe PASS |
| Transition window max | `0,02441` | `2e-3` | 12× oltre | — | — |
| First-difference max | `0,07031` | `3e-3` | 23× oltre | — | — |
| Worst per-caso/azione RMSE | `0,004824` | `5e-4` | 10× oltre | — | — |
| Predicted mean max abs | `0,8996` | `0,95` | — | — | PASS |

RMSE per caso: tutti in `[0,00444; 0,00463]` — errore **uniforme** sui sei
casi, senza patologie localizzate (a differenza di R9, dove l'observer
nominale era 4× peggio del resto: segnale a favore della tesi che il masking
a 18 feature elimina l'alias).

Ottimizzatore: obiettivo terminale `1,71e-5`, 3.078 closure LBFGS.

## Diagnosi

1. **I gate erano infattibili, non validati da alcun dry-fit.** Le soglie
   V12R12 erano ~10× più severe di ogni lineage precedente e ~12× sotto il
   miglior risultato offline mai registrato nel progetto (W1024
   gate-aligned: `0,00497`; V5 W256: `0,00304`). Nessuna architettura ha
   mai avvicinato `2,5e-4`. A differenza di V12R10, il cui obiettivo
   gate-aligned era stato validato da un dry-fit **prima** del freeze, i
   gate V12R12 codificavano una speranza, non una fattibilità dimostrata.
2. **Il costo del masking è reale ma moderato**: `0,00453` contro `0,00304`
   della W256 non mascherata (~1,5× su RMSE, ~2,2× sul max). Le 18 feature
   invarianti perdono informazione, ma il fit resta al livello che ha
   sempre superato i gate storici della famiglia (0,006 / 0,060 / 0,003).
3. **La domanda fisica resta senza risposta.** L'innovazione di V12R12 per
   la stabilità closed-loop non era la fedeltà offline: erano le feature
   invarianti (anti-alias) e il P1 candidate-exposed con shield prima del
   test puro. Entrambe restano non testate, perché il gate offline ha
   ucciso la pipeline prima della fisica.

## Strategia raccomandata per il successore (V12R13)

- Design **identico** a V12R12 (masking 18 feature, teacher da tape,
  P0→P1→P2→P3 hardest-first) con i fix pre-freeze già inclusi;
- **gate offline ricalibrati a livelli dimostrati fattibili** dalla
  riproduzione (che è, di fatto, il dry-fit del successore: il fit è
  deterministico e riprodurrà esattamente questi valori): ad es. RMSE
  ≤0,005, max ≤0,060, reset ≤0,003, per-caso ≤0,0055, transition ≤0,030,
  first-difference ≤0,080 — il gate offline torna al suo ruolo storico di
  sanity check, e il discriminatore reale torna a essere **P3 fisico**
  (`+0.20` per primo), dove appartiene;
- correggere il gap forense ricorrente: **persistere metriche e summary
  prima della valutazione del gate**, così un futuro FAIL lascia i numeri
  su disco (lezione ora ripetuta due volte: V12R3 fit_p2, V12R12 P0);
- nessuna nuova raccolta: stessi input pinnati (tape, H0, closure R10).

## File modificati o prodotti

- artefatti terminali V12R12 (immutabili): freeze, lock,
  `h0_v12r12_run_20260815/p0_fit/{corpus.npz, gate.json, failure.json}`;
- diagnostica successore: `v12r13/diagnostics/` (script + JSON riprodotto,
  hash sopra);
- questo report.

Nessun file di produzione è stato modificato dopo il freeze; nessun
namespace storico è stato toccato.

## Test e verifiche

- preflight, freeze `--verify` implicito nella pipeline e lock: PASS;
- esito terminale P0 letto da `gate.json`/`failure.json` reali;
- riproduzione deterministica in-memory: `gate_passed=false` con gli stessi
  7 check falliti del run canonico (conferma incrociata);
- zero reset/step ambiente, zero query teacher, zero update critic/PPO in
  tutta la sequenza.

## TODO

- [ ] Autorizzare la lineage V12R13 (design invariato, gate ricalibrati e
  dry-fit-validati, persistenza metriche pre-gate).
- [ ] Eseguirne la sequenza one-shot fino a P3: la domanda aperta —
  stabilità fisica a `+0.20` con feature invarianti + recovery
  candidate-exposed — si decide lì.
- [ ] Dopo un eventuale PASS P3: port Q3 → checkpoint-zero → morphology →
  preflight 12 EnvRunner → training 50 update (percorso già costruito).
