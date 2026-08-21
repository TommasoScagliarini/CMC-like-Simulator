# Verdetto lineage nativa V26 a iterazione 200: infrastruttura promossa, equivalenza con giugno NON raggiunta

Data: 2026-08-19

## Esito operativo

`_cont200_final` (151→200): 50/50 iterazioni, ~8 min/iter, **0 errori
off-grid, zero worker morti**. L'intera catena post-fix (129→200, 72
iterazioni) è girata senza un singolo incidente: i due bug corretti oggi
sono chiusi in produzione.

## Confronto formale coi criteri di giugno (ultime 20 iterazioni)

| Metrica | Giugno (fine) | Nativa V26 @200 | Rapporto |
|---|---:|---:|---:|
| Best return | 293,6 (iter 94) | 196,4 (iter 162) | 67% |
| Return medio ultime 20 | 268,6 | 173,5 | 65% |
| Len media ultime 20 | 476 | 401 | 84% |
| Return per step | 0,564 | 0,433 | 77% |

Mix terminazioni sulle 50 iterazioni finali (contatori onesti):
timeout swing **+3**, stance +6, eventi invalidi 0 (assorbiti come drop),
**penetrazione +301 (≈6/iter, piatta)**. Nota comparativa: anche giugno
aveva ≈5,3 penetrazioni/iter — ma i suoi episodi morivano più tardi e
il suo tracking per step era migliore. Il gap residuo NON è più un muro
strutturale (timeout ≈ zero, orizzonte raggiungibile): è **qualità di
tracking** (in primis lo smorzamento dei picchi di flessione del
ginocchio, già visibile nei plot del 18/08) più penetrazioni
mid-episodio.

**Verdetto sul criterio «stesso risultato»: NON soddisfatto a budget 200.**
La curva è in plateau da ~40 iterazioni (best 196 a iter 162, ultime 20 a
173,5): altre iterazioni da sole difficilmente colmano un gap del 35%.

## Lettura e opzioni

La lineage attuale porta una storia sporca: 100 iterazioni nel regime
strozzato (tetto ~250 step) + 28 nel regime crash-storm (campionamento
distorto) + 72 pulite. La policy ha consolidato abitudini apprese in
condizioni sbagliate (ginocchio conservativo, atterraggi imperfetti).

- **(A) Training fresh nativo V26 con tutti i fix** — raccomandata: 100
  iterazioni da zero nel regime corretto (drop-mode, swing 2,6, orizzonte
  robusto) — l'analogo pulito dell'esperimento di giugno, senza il
  confound della storia. Prerequisito già in lista: fix del probe
  (`train_ppo_mlp.py:1408` → `make_cmc_env`), che dà anche lo schema
  actor pieno a 43 feature. Costo: ~13–14 h al ritmo certificato.
- **(B) Reward shaping mirata** (atterraggio/penetrazione) sul checkpoint
  corrente: attacca il sintomo residuo ma eredita la storia; seconda
  linea.
- **(C) Solo più iterazioni**: sconsigliata — plateau e penetrazione
  piatta.

## File

- run: `runs/training/MLP_imitation_native_v26_08-19-2026_swing2p6_rejectcontinue_cont200_final/`
  (checkpoint_best iter 162, checkpoint_last iter 200);
- catena provenance: baseline 08-17 → smoke10 → cont200 (crash-storm,
  fermata a 128) → verify3 → cont150 → cont200_final.

## Test e verifiche

- 0 off-grid su tutte le sessioni Ray post-fix; durate iterazione da
  tensorboard; confronto su jsonl reali di entrambe le run.

## TODO

- [ ] Decisione utente: fresh run (A) vs reward shaping (B); per (A)
  applicare prima il fix probe (2 righe + verifica n_actor=43).
- [ ] Eventuale rollout + 7 plot del best 200 per confronto visivo.
- [ ] Daily 2026-08-18/19 da consolidare al prossimo `end_day`.
