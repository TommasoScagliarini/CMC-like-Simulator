# H0 V12R16 — verdetto P3: il killer storico `+0.20` è PASS, seed 127 terminale

Data: 2026-08-16

## Esito

La lineage V12R16 ha completato l'intero percorso offline pre-validato
(P0 PASS → P1 PASS 18/18 con 8.404 righe → P2 PASS, candidato
`AB06_H0_V12R16_V26_INVARIANT_SAFE_TEACHER:ecdda4286eeadaaf`) ed è arrivata
— **prima lineage della saga** — al giudizio fisico P3: sei rollout puri,
senza teacher, blend o scudi.

| Caso P3 (ordine risk-first) | Esito | Dati |
|---|---|---|
| `deterministic_offset_plus_0p20` | **PASS** | 500/500 step, `episode_time_limit`, 2 cicli validi, penetrazione max **24,901 mm**, zero clipping |
| `stochastic_nominal_seed_127` | **FAIL terminale** | 202 step, `grf_penetration` **25,176 mm** (176 µm oltre), 0 cicli, zero clipping |

Pipeline: `FAIL_H0_V12R16_P3_PURE_PIPELINE_TERMINAL`. I quattro casi restanti
non si sono aperti. Q3, checkpoint-zero, morphology e training restano chiusi.

## Il PASS storico

Il caso `+0.20` — che aveva ucciso R6 a 179 step, R10 a 212, R11 a 209, e
che **nemmeno H0 originale reggeva su V26** — è stato completato per intero
da una policy autonoma, al primo tentativo puro della lineage. La tesi di
design è confermata sul discriminatore: masking a 18 feature invarianti
(anti-alias) + distillazione dai tape sicuri + recovery candidate-exposed
rendono stabile il punto storicamente letale.

## Il FAIL e la sua anatomia

Seed 127 muore a **step 202 in ogni configurazione con ≥50% di influenza del
candidato**: blend 0,50 sotto latch (R13/R15: 202), blend 0,75 sotto latch
(R15: 202), policy pura (R16: 202). Sopravvivono solo il teacher puro del
tape (500/500) e il blend a 25% di candidato (500/500 con latch ~50% degli
step). Lo stesso evento stocastico (~2,0 s in quella realizzazione di
rumore) sconfigge qualunque cosa il candidato guidi davvero, latch incluso.

## L'intuizione strutturale (il dato più importante del verdetto)

Il paradigma del **teacher da tape congelato ha un tetto di etichettatura
esattamente dove serve di più**. Le label P1 sono `tape(case, step)`: valide
finché gli stati del candidato restano vicini a quelli del tape. Al punto di
divergenza di seed 127, gli stati del candidato si allontanano dal tape e la
lookup per (caso, step) etichetta lo stato *del tape*, non quello *del
candidato*: le righe di "recovery" lì sono off-policy. Coerentemente, il
worst per-traiettoria del fit P2 (0,0068, il peggiore del corpus) sta
proprio su quelle traiettorie troncate — **già amplificate 2,5× dalla
normalizzazione di massa** — e il candidato muore comunque nello stesso
punto. Più round della stessa raccolta non possono superare questo tetto; e
un teacher vivo V26-compatibile non esiste (V12R11: H0 diretto fallisce).

## Nota decisiva per l'obiettivo training-ready

Il caso che fallisce è **fuori dalla distribuzione di training**: il regime
del 15/07 campiona esclusivamente dai tre start deterministici (−0,20,
nominale, +0,20) — e il `+0.20` è PASS, insieme (in P1 sotto blend spinti)
agli altri due. Seed 127 è un caso di *qualification* held-in, non uno start
di training. Una qualifica dichiaratamente ristretta non toccherebbe la
copertura della distribuzione su cui PPO campiona.

## Opzioni per la decisione

- **(A) Capacità W512 mascherata**: dry-fit deterministico gratuito sul
  corpus esistente per misurare se l'errore sulle traiettorie di recovery
  scende in modo sostanziale; nuova lineage solo se i numeri lo
  giustificano. Onestà: il tetto è nella qualità delle label, non solo
  nella capacità — probabilità moderata.
- **(B) Altri round di raccolta identici**: sconsigliata — stesse label
  off-policy al punto di divergenza, stesso tetto.
- **(C) De-scope dichiarata della qualification**: sei casi con seed 127
  sostituito (es. seed 129/130) o qualifica 5/6 con seed 127 documentato
  come *known-open boundary*; la catena prosegue (P3 → Q3 → checkpoint-zero
  → morphology → training) con una limitazione esplicita e tracciata. È
  la via più diretta all'obiettivo dichiarato (training-ready con le tre
  feature), scientificamente difendibile perché la distribuzione di
  training resta interamente coperta dai casi PASS.
- **(D) Cambiare paradigma di teacher** (oltre il tape congelato): richiede
  ricerca nuova, fuori scala giornaliera.

## File e artefatti

- `validation/v12r16/` completo: freeze/lock fitter, P1, P3; run root
  `h0_v12r16_run_20260816/` con p0_fit, p1_candidate_exposed (receipt PASS),
  p2_fit (receipt PASS, candidato congelato), p3_development (caso `+0.20`
  PASS + forense completa del caso 127 + ledger terminale).

## Test e verifiche

- esiti letti dai receipt/gate/summary/ledger reali;
- suite v12r16 pre-freeze: 84/84 PASS; ruff PASS;
- determinismo confermato lungo tutta la catena (P1 bit-identico a R15).

## TODO

- [ ] Decisione dell'utente tra le opzioni (A)/(C) — o loro combinazione
  (dry-fit A per informare C).
- [ ] In caso di (C): nuova lineage con matrice di development dichiarata,
  poi port filiera Q3 → checkpoint-zero → morphology → preflight →
  training.
- [ ] Commit checkpoint R13–R16.
