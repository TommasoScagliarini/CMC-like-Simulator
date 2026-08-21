# Training imitativo nativo V26 (100 iter) — verdetto: tracking quasi pari, episodi dimezzati dal muro dell'heel-qualification

Data: 2026-08-18

## Esito operativo

Run `MLP_imitation_native_v26_08-17-2026_grfsoft_knee1_ankle2_100iter`
completata: **100/100 iterazioni, zero worker morti, zero crash, zero
abort** — la policy `terminate` ha assorbito 549 eventi invalidi organici
lungo tutta la run. La modifica runtime ha retto perfettamente ~18 ore di
training.

## Confronto formale con la baseline di giugno (criteri fissati)

| Iter | Return giugno | Return V26 | Len giugno | Len V26 |
|---:|---:|---:|---:|---:|
| 10 | 47 | 17 | 428 | 142 |
| 25 | 127 | 80 | 389 | 210 |
| 50 | 232 | 105 | 456 | 234 |
| 75 | 270 | 121 | 501 | 247 |
| 100 | 272 | 124 | 478 | 246 |

Best: 294 vs 141 (−52%); media ultime 20: 269 vs 122 (−55%); len max:
501 vs 299.

**Verdetto sul criterio "stesso risultato": NON soddisfatto sui numeri
grezzi.** Ma la decomposizione cambia la lettura:

- **return per step: ~0,50 vs ~0,57 (≈88% di giugno)** — la qualità di
  tracking per passo è quasi pari; il gap è quasi interamente **lunghezza
  episodio** (~250 vs ~480 step);
- **la penetrazione è stata RISOLTA dall'apprendimento**: terminazioni per
  blocco di 25 iter in calo 248 → 150 → 74 → 58 — la policy nativa ha
  imparato organicamente ciò che nessuna distillazione era riuscita a
  ottenere;
- il muro è **uno e solo uno**: `phase_timeout_swing`, dominante e piatto
  per tutta la run (378/250/375/351 per blocco, 1.354 totali) + un
  soffitto secondario di `invalid_binary_event` (~150/blocco, 549 totali).

Giugno, per contrasto: **zero timeout di fase** e 606 episodi a orizzonte
pieno.

## Diagnosi

La FSM attore termina lo swing dopo `swing_hard_timeout_s = 1,30 s` senza
heel strike. Sotto il detector legacy l'HS veniva concesso liberamente
(soglia analogica permissiva); sotto V26 l'HS richiede contatto di tallone
stabile heel-qualified. La policy traccia il riferimento al ~88% ma i suoi
atterraggi residualmente imperfetti non superano la qualificazione → nessun
HS → timeout a ~2,5 s → episodio troncato. Effetto perverso: il tetto a
~250 step **impedisce alla policy di allenarsi sulla parte successiva del
segmento** (trappola curricolare), e infatti il trend dei timeout non
migliora con l'apprendimento — è strutturale, non lentezza.

Nota: il riferimento eseguito perfettamente SUPERA la qualificazione (i
tape teacher su V26 hanno HS regolari): il problema è il residuo di
tracking all'atterraggio, non un'impossibilità.

## Leve disponibili (decisione richiesta)

- **(A) Rimuovere i due soffitti artificiali in training** — raccomandata:
  1. `phase_swing_hard_timeout_s`: 1,30 → ~2,60 s per il training (il
     parametro esiste già nell'env config; manca solo il passthrough dal
     trainer, 2 righe). Compensazione di principio: il legacy risolveva gli
     swing anomali coi propri timeout interni *senza* terminare, V26 non ha
     quel meccanismo;
  2. eventi invalidi: policy `reject_continue` (già implementata, un flag)
     al posto di `terminate` — lo spirito "assorbi e prosegui" di giugno.
  Poi **proseguire dal checkpoint** (`--resume-from checkpoint_last`) con
  uno smoke da 10 iter per verificare che la lunghezza episodi sfondi i
  250, e a seguire la continuazione piena.
- **(B) Semplicemente più iterazioni**: debole — il trend dei timeout è
  piatto.
- **(C) Reward per l'atterraggio di tallone**: modifica di design della
  reward, da tenere come seconda linea se (A) non basta.

## File coinvolti

- run: `Trajectory Generator/runs/training/MLP_imitation_native_v26_08-17-2026_grfsoft_knee1_ankle2_100iter/`
  (checkpoint_best/last, train_iterations.jsonl, summary.json);
- nessuna modifica sorgente in questa milestone (analisi).

## Test e verifiche

- confronto curve su dati per-iterazione reali di entrambe le run;
- mix e trend delle terminazioni per blocco verificati sui log;
- zero errori worker verificato sul log driver della run.

## TODO

- [ ] Decisione utente sulla leva (A) (timeout swing in training +
  `reject_continue`) e sulla modalità di prosecuzione (resume vs fresh).
- [ ] Dopo il fix: smoke 10 iter dal checkpoint, poi continuazione e
  ri-confronto sui criteri; a pari lunghezza episodi, valutare anche il
  rollout di tracking (RMSE per giunto) contro i plot `06_24`.
- [ ] A baseline raggiunta: catena di luglio (critic warmup → qualifica →
  checkpoint-zero → morphology → training ex-novo + corridor).
