# Diagnosi della telemetria del training con oscuramento

Analisi esclusivamente offline dei JSONL e TensorBoard già salvati; nessuna modifica di produzione, training o simulazione. Script e dati aggregati nella stessa directory.

## Risultato principale

L'imitazione negli episodi oscurati è già scarsa nel primo blocco e rimane quasi piatta. Il degrado della media complessiva riflette soprattutto una quota crescente di episodi difficili; non dimostra da solo una perdita progressiva e totale di capacità. Si osserva comunque un deterioramento degli episodi visibili, soprattutto ankle, e una crescita molto forte delle code della deviazione standard della policy durante il sampling.

| Campioni | Episodi visibili / oscurati | RMSE visibile knee / ankle | RMSE oscurato knee / ankle | Reward per step visibile / oscurato |
|---|---:|---:|---:|---:|
| Baseline update 81–90 | 81 / 0 | 7,33° / 3,58° | — | 0,555 / — |
| 25% oscuramento | 171 / 53 | 7,83° / 4,29° | 14,27° / 9,16° | 0,525 / 0,284 |
| 50% oscuramento | 115 / 132 | 7,47° / 5,16° | 14,38° / 9,40° | 0,502 / 0,289 |
| 75% oscuramento | 49 / 212 | 8,50° / 5,29° | 14,13° / 9,61° | 0,489 / 0,304 |
| 100% oscuramento | 0 / 266 | — | 14,22° / 9,23° | — / 0,329 |

RMSE calcolati come `sqrt(sum(sum_squared_rad2)/sum(count))`, convertiti in gradi: q fisico contro target imitativo su tutti i segmenti completati degli episodi registrati. Non sono medie degli RMSE dei singoli episodi. La baseline include episodi con prima versione policy >=80 e ultima <=89, cioè i dati raccolti per gli ultimi dieci update che conducono a CP90. Questi dati non sono rollout deterministici di un checkpoint fisso.

Reward esatta su tutti i passi dei batch, da contatori TensorBoard `console/reward_sum_current / console/reward_count_current`: 0,5505 baseline; 0,4736 / 0,3947 / 0,3438 / 0,3307 nei quattro blocchi. La differenza rispetto alla tabella è dovuta agli episodi incompleti non presenti nella telemetria episodica.

I passi effettivamente visibili sono il 77,72%, 49,14%, 20,48% e 0%. I ritorni non sono direttamente confrontabili tra blocchi come ranking del checkpoint: cambia la distribuzione degli ingressi.

## Andamento interno ai blocchi

RMSE oscurati nelle prime cinque e ultime cinque policy di sampling di ciascun blocco (attribuzione alla versione presente all'ultimo passo dell'episodio):

| Blocco | Prime 5 knee / ankle | Ultime 5 knee / ankle | N episodi prime / ultime |
|---|---:|---:|---:|
| 25% | 13,90° / 8,58° | 13,57° / 9,43° | 9 / 9 |
| 50% | 14,86° / 9,61° | 14,11° / 9,23° | 17 / 32 |
| 75% | 14,15° / 9,12° | 14,32° / 9,93° | 37 / 38 |
| 100% | 14,58° / 9,57° | 14,12° / 8,90° | 49 / 56 |

Non emerge un recupero cinematico consistente. I piccoli sottogruppi iniziali non consentono conclusioni statistiche forti. L'89–97% degli episodi attraversa almeno un aggiornamento di policy: non sono valutazioni di checkpoint singoli.

## Esplorazione: crescita verificata delle code

Gli output registrati sono `[mu_knee,mu_ankle,logstd_knee,logstd_ankle]`. La classe effettiva è il fallback RLlib `TorchDiagGaussian`; `from_logits` in `ray/rllib/core/distribution/torch/torch_distribution.py` usa `exp(log_std)` senza clamp. Il modulo custom non introduce clamp dei logits. Ricostruendo la densità normale dei raw action salvati si ottiene `action_logp` con errore massimo inferiore a 1,9e-6 su tutte le righe: logits e azioni sono allineati.

| Campioni | Mediana sigma knee / ankle | P99 sigma knee / ankle | Frazione sigma knee >10 |
|---|---:|---:|---:|
| Baseline | 1,12 / 1,03 | 10,34 / 13,47 | 1,11% |
| 25% | 1,06 / 1,04 | 21,41 / 25,26 | 3,85% |
| 50% | 1,06 / 1,08 | 76,18 / 86,54 | 9,03% |
| 75% | 1,03 / 0,98 | 198,92 / 94,16 | 13,27% |
| 100% | 1,06 / 0,96 | 1063,20 / 159,07 | 18,07% |

Unità di sigma: azione normalizzata grezza, prima del clipping; non gradi, rad o N·m. Questi dati coprono solo le ultime fino a 50 righe per episodio, non tutti i passi di training. Sono quindi sovrarappresentati gli stati finali e di fallimento. Tuttavia la crescita persiste usando soltanto episodi terminati regolarmente per time limit: P99 knee 7,85 → 14,93 → 27,58 → 58,82 → 92,87.

Esempio verificabile in `tail_sigma_distribution.json`: policy 188, episodio oscurato `p95674_1790796709689890000:w10:e0:b47860f389d94580bb48a73a2c423cd2`, step 374, t=16,73. `logstd_knee=8.88894`, sigma=7251,36, mu=-0,0647, raw action knee=1093,61 e ankle=-108,32; endpoint applicato `[0,-0.7]`. L'episodio termina per penetrazione allo step 380, return 107,48. Un esempio senza arresto anticipato è policy 186, episodio `p95669_1790796709695798000:w5:e0:1e032cf93f0e40b0a627539192009b6d`: step 496 sigma knee=917,22, raw action=-233,95; completa 500 passi con return 164,22.

È evidenza di varianza estrema in alcune regioni e azioni che raggiungono i limiti del comando. Non dimostra da sola che la varianza causi il degrado: gli stati visitati cambiano, il controller e governor filtrano gli endpoint, e i rollout deterministici utilizzano la media.

## Learner, reward e terminazioni

- KL media per blocco: 0,0093 / 0,0086 / 0,0096 / 0,0120 (target 0,01). Picchi 0,0382 all'update 156 e 0,0309 al 182. Non c'è una divergenza sistematica della KL media.
- Entropia media: baseline 1,753, poi 1,815 / 1,938 / 2,086 / 2,135. La media non rende visibili da sola le code estreme della sigma.
- Explained variance del critic: 0,571 / 0,756 / 0,733 / 0,609. Nessun collasso a zero; il critic resta soggetto al problema bootstrap già auditato, non isolabile con queste sole metriche.
- I termini effettivi `ankle_positive_torque_term`, `knee_torque_rate_term`, `tuning_quality_term` sono sempre zero: il miglioramento delle coppie non segue l'introduzione di questi premi diretti nel pilot.
- Le componenti reward TensorBoard sono medie mobili di 100 passi per worker (`tb_logging.py:547`), non decomposizioni esatte dell'intero batch. In queste finestre imitation score passa 0,832 → 0,764 / 0,698 / 0,656 / 0,624; served imitation score 0,733 → 0,654 / 0,583 / 0,538 / 0,508. Tracking loss resta piccola, 0,0017–0,0030: l'inseguimento del comando servito è migliore dell'imitazione del target.
- Episodi conclusi per time limit: 136/224, 93/247, 85/261, 76/266. Nel blocco completamente oscurato 185/266 terminano per penetrazione, 4 per divergenza knee e 1 per timeout stance; la durata media è 374,8 passi. Completare un rollout deterministico di 500 passi non implica robustezza durante sampling.

## Artefatti

- `summary.json`: gruppi episodi e metriche learner.
- `iterations.csv`, `episodes.csv`: dati aggregati di dettaglio.
- `tensorboard_summary.json`, `tensorboard_rolling_metrics.csv`: componenti reward e diagnostiche salvate.
- `tail_sigma_distribution.json`: quantili, controlli logp, esempi e gruppi time limit/failure/visibile/oscurato.
- `training_diagnosis.png`: quattro pannelli del confronto, ispezionati visivamente.
- Script riproducibili: `analyze_training.py`, `extract_tensorboard.py`, `analyze_tail_sigma.py`, `plot_training.py`.
