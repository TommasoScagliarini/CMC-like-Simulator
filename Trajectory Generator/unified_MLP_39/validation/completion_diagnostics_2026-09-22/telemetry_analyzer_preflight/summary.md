# Diagnosi offline degli episodi conclusi

Episodi letti: **1**; completamento: **100.0%**. Record duplicati: 0; righe invalide: 0.

| Esito | Episodi | Durata media [s] | Picco penetrazione medio [mm] | RMSE ginocchio [°] | RMSE caviglia [°] | Step con clipping |
|---|---:|---:|---:|---:|---:|---:|
| full | 1 | 5.000 | 21.223 | 10.233 | 3.981 | 52.8% |

## Motivi e fasi terminali

- **full** — motivi: `{"episode_time_limit": 1}`; fasi: `{"SWING_AFTER_TO": 1}`.

## Errori terminali e ultimi 50 step

| Esito | Errore finale ginocchio [°] | Errore finale caviglia [°] | RMSE tail ginocchio [°] | RMSE tail caviglia [°] |
|---|---:|---:|---:|---:|
| full | 21.548 | 8.121 | 14.136 | 5.315 |

Errori terminali assoluti medi. Il dettaglio firmato è nel JSON. Prima attività di penetrazione nella coda:

| Esito | Soglia | Episodi con attività | Transizioni osservate | Già attiva al bordo / censura |
|---|---|---:|---:|---:|
| full | above_0_mm | 1 | 0 | 1 |
| full | above_10_mm | 1 | 0 | 1 |
| full | above_20_mm | 0 | 0 | 0 |
| full | above_30_mm | 0 | 0 | 0 |
| full | penalty_active | 0 | 0 | 0 |

## Eventi vicini alla fine

Conteggi yes / no / unknown; “full” termina al limite temporale. Sono associazioni descrittive.

| Evento | Finestra [step] | full | failed |
|---|---:|---|---|
| resync | 5 | 0 / 1 / 0 | 0 / 0 / 0 |
| resync | 10 | 0 / 1 / 0 | 0 / 0 / 0 |
| resync | 25 | 0 / 1 / 0 | 0 / 0 / 0 |
| resync | 50 | 0 / 1 / 0 | 0 / 0 / 0 |
| hs_cancel | 5 | 0 / 1 / 0 | 0 / 0 / 0 |
| hs_cancel | 10 | 0 / 1 / 0 | 0 / 0 / 0 |
| hs_cancel | 25 | 0 / 1 / 0 | 0 / 0 / 0 |
| hs_cancel | 50 | 0 / 1 / 0 | 0 / 0 / 0 |
| policy_switch | 5 | 0 / 1 / 0 | 0 / 0 / 0 |
| policy_switch | 10 | 0 / 1 / 0 | 0 / 0 / 0 |
| policy_switch | 25 | 0 / 1 / 0 | 0 / 0 / 0 |
| policy_switch | 50 | 0 / 0 / 1 | 0 / 0 / 0 |

## Finestre della versione policy terminale

| Versioni pesi | Episodi | Completamento | Durata media [s] |
|---|---:|---:|---:|
| 40–44 | 1 | 100.0% | 5.000 |

| Versione durante l’episodio | Episodi | Completamento | Durata media [s] |
|---|---:|---:|---:|
| unknown | 1 | 100.0% | 5.000 |

Strati aggiuntivi in `summary.json`: versione iniziale/finale, coppia iniziale→finale, policy stabile/con cambi, durata×versione e qualità dei dati. Dettaglio episodio in `per_episode.jsonl`.

## Limiti

- Analisi descrittiva di episodi conclusi e scritti: episodi ancora aperti, processi morti o scritture perse non sono inclusi. La percentuale non coincide automaticamente con le metriche RLlib per iterazione.
- Il 38.8% storico è il contesto della diagnosi, non un denominatore di questa nuova telemetria. Confrontare finestre di policy ed esposizione prima di attribuire un miglioramento.
- Episodi lunghi hanno più opportunità di clipping, resync, cancellazioni e cambi di policy. Sono riportati tassi per 100 step, durata e strati per versione; questi confronti restano confusi dalla durata e dalla selezione degli episodi sopravvissuti.
- RMSE episodio è calcolato sugli step osservati dell’intero episodio; il RMSE tail riguarda al massimo gli ultimi 50. Gli episodi falliti coprono intervalli più brevi: gli RMSE non hanno supporto temporale identico.
- La precedenza temporale di resync, cancellazione o cambio policy non dimostra causalità. I conteggi entro N step confrontano la fine degli episodi falliti con la fine amministrativa dei completi; non stimano rischio per singolo evento.
- Onset indica la prima attività osservata nella coda, non necessariamente l’inizio dell’evento nell’intero episodio; attività già presente all’inizio è censurata a sinistra. Le soglie 0/10/20/30 mm sono diagnostiche; penalty_active usa la penalità effettivamente registrata.
- I cambi di policy temporizzati sono rilevati solo tra righe consecutive nella coda. 50 righe coprono 49 intervalli: l’assenza di cambi nei 50 step precedenti può restare unknown. Le versioni pesi non sono automaticamente numeri di iterazione.
- I record legacy privi del riepilogo policy_versions consentono solo controlli nella coda: first e switch_count dell’intero episodio restano sconosciuti. Nessuna ipotesi sul numero di worker.
