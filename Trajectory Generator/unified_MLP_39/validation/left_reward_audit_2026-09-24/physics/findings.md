# Audit fisico della reward LEFT best72

Analisi offline di 11 episodi già salvati; nessuna simulazione o modifica del training.

La reward viene ricostruita con `compute_reward` e configurazione del rollout. Le somme includono i terminali e coincidono con gli episode_return entro 1e-8. La riga hard finale di seed127 sigma1 viene esclusa dalle statistiche fisiche perché ha timestamp duplicato e contatti azzerati.

| Caso | Esito | Passi fisici | Reward media, prefisso 303 | RMSE knee/ankle, prefisso 303 [°] |
|---|---|---:|---:|---:|
| deterministic | episode_time_limit | 500 | 0.541 | 6.456 / 3.875 |
| noise_0.5_seed123 | episode_time_limit | 500 | 0.662 | 6.347 / 2.505 |
| noise_0.5_seed124 | episode_time_limit | 500 | 0.654 | 5.810 / 2.997 |
| noise_0.5_seed125 | episode_time_limit | 500 | 0.648 | 5.472 / 2.541 |
| noise_0.5_seed126 | episode_time_limit | 500 | 0.687 | 5.513 / 2.428 |
| noise_0.5_seed127 | grf_penetration | 469 | 0.644 | 5.545 / 3.094 |
| noise_1_seed123 | episode_time_limit | 500 | 0.581 | 7.087 / 3.723 |
| noise_1_seed124 | episode_time_limit | 500 | 0.597 | 5.663 / 4.178 |
| noise_1_seed125 | grf_penetration | 303 | 0.419 | 7.300 / 5.176 |
| noise_1_seed126 | episode_time_limit | 500 | 0.623 | 5.982 / 2.910 |
| noise_1_seed127 | grf_penetration_hard | 465 | 0.541 | 7.379 / 4.151 |

## Finestre finali appaiate: fino a 20 passi fisici prima del fallimento

Tutti i confronti nella stessa finestra usano esattamente gli stessi tempi. Per il fallimento sigma0.5 seed127, il confronto comune si ferma a 465 e copre 16 passi, perché sigma 1 è già terminato.

| Caso fallito | Caso confrontato | N | Reward media | Imitazione ponderata | Penalità penetrazione | Penalità momento | Pen. max [mm] |
|---|---|---:|---:|---:|---:|---:|---:|
| noise_1_seed125 | deterministic | 20 | -0.343 | 0.784 | 0.005 | 0.965 | 21.83 |
| noise_1_seed125 | noise_1_seed125 | 20 | -0.818 | 0.537 | 0.118 | 0.936 | 28.41 |
| noise_1_seed125 | noise_0.5_seed125 | 20 | 0.331 | 0.875 | 0.000 | 0.374 | 17.36 |
| noise_1_seed127 | deterministic | 20 | -0.349 | 0.439 | 0.000 | 0.637 | 20.11 |
| noise_1_seed127 | noise_1_seed127 | 20 | 0.485 | 0.773 | 0.059 | 0.023 | 27.40 |
| noise_1_seed127 | noise_0.5_seed127 | 20 | 0.655 | 0.867 | 0.013 | 0.017 | 23.68 |
| noise_0.5_seed127 | deterministic | 16 | -0.124 | 0.422 | 0.000 | 0.400 | 20.11 |
| noise_0.5_seed127 | noise_1_seed127 | 16 | 0.412 | 0.718 | 0.074 | 0.029 | 27.40 |
| noise_0.5_seed127 | noise_0.5_seed127 | 16 | 0.622 | 0.840 | 0.017 | 0.021 | 23.68 |

Gli snapshot numerici completi (angoli, errori actual/served, velocità, fase, forza, momento, reward) sono in `metrics.json`; `steps.csv` conserva tutte le righe fisiche valide.

## Penetrazione oltre soglia e reward immediata

Esclusi i primi 0,2 s. Gli episodi completi possono superare 20 mm e recuperare: la categoria descrive una penalità attiva, non classifica automaticamente un fallimento.

| Rumore originale | Passi oltre 20 mm | Reward positiva | Reward >0,5 | Reward media su questi passi |
|---|---:|---:|---:|---:|
| 123 | 20 | 20 | 11 | 0.460 |
| 124 | 27 | 21 | 4 | 0.167 |
| 125 | 26 | 22 | 7 | -0.160 |
| 126 | 0 | 0 | 0 | - |
| 127 | 9 | 7 | 6 | 0.346 |

## Interpretazione

- Seed125 sigma1: il momento GRF molto negativo attiva una forte penalità prima che la penetrazione superi 20 mm. Nella finestra finale la reward è già fortemente negativa; il caso non supporta una mancata penalizzazione locale.
- Seed127 fallisce con entrambe le scale, con momento GRF positivo nella salita finale della penetrazione: aumentare solo anti-flip non affronta questo meccanismo.
- Le ultime osservazioni sotto 20 mm precedono l’ultima osservazione fisica di 40 ms (seed125 sigma1), 30 ms (seed127 sigma1, poi hard failure) e 70 ms (seed127 sigma0.5). La salita finale è rapida; questa osservazione motiva verificare il margine anticipatorio, ma non dimostra che anticipare la penalità risolva il problema.
- Seed127 sigma1 negli ultimi 20 passi fisici ha reward media +0,485 contro −0,349 del deterministico che sopravvive: il deterministico imita peggio e paga maggiore anti-flip. Nei singoli ultimi passi del collasso la reward diventa però negativa; inoltre il fallimento toglie reward future. Il confronto locale non dimostra che la reward globale preferisca il fallimento.
- La penetrazione non costa direttamente sotto 20 mm; subito oltre soglia il costo quadratico è ancora piccolo. Questo non è un bug, ma una scelta di margine/sensibilità da valutare.
- Confronti appaiati tengono fissi tempo e target, ma non stato né storia delle azioni; non provano causalità di un peso della reward.

## Limiti

- Same Gaussian draws do not imply same absolute action perturbations: each trajectory has its own state-dependent policy standard deviation.
- Failure windows compare different action/state histories and are associations, not interventions isolating a joint or reward coefficient.
- Contact values come from physical reward diagnostics, not detector_sensors geometrical penetration.
- Scalar reconstruction includes the hard terminal row; physical statistics exclude its duplicated timestamp and zeroed contact fields.
- Means across complete/failing full trajectories are descriptive only; use shared 303-step prefixes or explicitly paired time windows for comparisons.
- Thresholds of 15, 20, 24 mm and 800 N are diagnostic categories, not evidence that all included states are failures.
- Immediate positive reward does not establish that a behavior has favorable discounted return; episode termination removes future reward.
- Only one learned policy, one start and five noise seeds; no independent training replicates or statistical equivalence test.
