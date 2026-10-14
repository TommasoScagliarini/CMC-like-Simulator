# Distribuzione reset IK — audit offline best47

Data: 2026-10-12. Nessun training, rollout, test nativo o cambiamento alla produzione.

Run: `Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01`. Il best è logical47; il run ha 50 nuovi update (logical2–51), riprendendo logical1.

Tutte le 50 righe registrano 1.536 transizioni reali per offset e PASS sia sul campionamento sia sul batch Learner: 76.800 per offset, 230.400 totali. Fino a best47: 46 update, 70.656 transizioni per offset.

| Offset [s] | Tempo IK assoluto [s], ricostruito | Runner assegnati | Nuove transizioni | Episodi completati registrati |
|---:|---:|---|---:|---:|
| 1.756870983805 | 13.746870983805 | [1, 4, 7, 10] | 76800 | 152 |
| 1.956870983805 | 13.946870983805 | [2, 5, 8, 11] | 76800 | 152 |
| 2.156870983805 | 14.146870983805 | [3, 6, 9, 12] | 76800 | 152 |

La lifetime nominale finale è 80.896: include 4.096 step ereditati dal rewarm critic precedente; non indica uno sbilanciamento dei nuovi update. Le altre lifetime sono 76.800.

Il sampler assegna deterministicamente `(worker_index-1+vector_index)%3`; quattro runner per offset. `random_init:false`: ogni runner riusa lo stesso tempo IK a ogni reset. Non è presente jitter su q/qdot. L'interleaving ha run-length massimo1 per la condizione, ma non rende indipendenti gli stati iniziali.

q/qdot riportati nel CSV sono calcolati offline usando il codice d'interpolazione corrente, IK corrente, filtro6Hz ordine4/dt1ms. Sono aspettative dai file attuali, non stati reset misurati durante il run storico. SEA motor angle/speed sono inizializzati alla posizione/velocità del giunto dal reset corrente.

| Offset [s] | Knee q [rad] | Knee qdot [rad/s] | Ankle q [rad] | Ankle qdot [rad/s] |
|---:|---:|---:|---:|---:|
| 1.756871 | -0.63548988 | 4.01099301 | -0.00385096 | 0.30246347 |
| 1.956871 | -0.17419569 | -0.04177851 | 0.05182407 | -0.71653207 |
| 2.156871 | -0.23804792 | -0.09213500 | 0.04245473 | 0.74427666 |

Precedenti J19B 6/6 e J19C 3/3 sono registrati separatamente nel JSON: i tre start deterministici e i seed stocastici nominali qualificavano l'actor J19 precedente, non best47 e non una prova odierna.

Limiti:

- The training run contains aggregated offset/episode statistics, not a persisted per-reset q/qdot/seed/time series. Reset counts and state equality across successive episodes cannot be proved from transition counts alone.
- Recorded completed episodes are end-callback counts, not the count of reset() calls: initial and final unfinished episodes differ, and historical per-runner episode IDs are absent.
- No runtime_manifest/progress.csv/result.json/reset telemetry file was found under the run. Summary/history, resolved YAML, and per-iteration guards are the authoritative available evidence.
- Current setup/IK/config and current sampler/reset source are SHA-pinned here, but their identity with files executed in September is not independently pinned by the inspected run metadata. Reconstructed q/qdot are current-file expectations, not historical measured reset states.
- Sampling interleaving decorrelates condition order in minibatches, not the physical reset state. Three fixed IK times span 0.4s of one AB06 trajectory; this is not broad initial-state randomization or statistical independence.
- Historical J19 qualification used an earlier actor and sigma0.005/held noise where declared; it does not establish robustness of the PPO best47 actor today.

Fonti: SHA256 e path completi nel JSON; righe50×3 nel CSV. Riferimenti codice:

- `env_factory.py:52–110`: assegnazione deterministica e callback prima del reset.
- `osim_trj_cmc_like.py:1786`, `:2540–2609`: seed/reset, tempo e inizializzazione episodio.
- `simulation_runner.py:337–390`, `:1796–1843`: IK q/qdot e SEA reset.
- `tb_logging.py:332–364`, `:458–505`: contatori transizioni corrente/lifetime ed episodi completati.
- `train_ppo_mlp.py:1134–1195`: per_start_metrics aggregati.
