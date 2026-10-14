# Diagnosi offline degli episodi conclusi

Episodi letti: **86**; completamento: **62.8%**. Record duplicati: 0; righe invalide: 0.

| Esito | Episodi | Durata media [s] | Picco penetrazione medio [mm] | RMSE ginocchio [°] | RMSE caviglia [°] | Step con clipping |
|---|---:|---:|---:|---:|---:|---:|
| full | 54 | 5.000 | 23.361 | 7.548 | 4.011 | 0.0% |
| failed | 32 | 3.790 | 28.987 | 6.529 | 3.867 | 0.0% |

## Motivi e fasi terminali

- **full** — motivi: `{"episode_time_limit": 54}`; fasi: `{"SWING_AFTER_TO": 41, "STANCE_AFTER_HS": 13}`.
- **failed** — motivi: `{"grf_penetration": 28, "grf_penetration_hard": 4}`; fasi: `{"STANCE_AFTER_HS": 32}`.

## Errori terminali e ultimi 50 step

| Esito | Errore finale ginocchio [°] | Errore finale caviglia [°] | RMSE tail ginocchio [°] | RMSE tail caviglia [°] |
|---|---:|---:|---:|---:|
| full | 10.892 | 5.276 | 10.058 | 4.786 |
| failed | 4.558 | 8.663 | 5.604 | 4.720 |

Errori terminali assoluti medi. Il dettaglio firmato è nel JSON. Prima attività di penetrazione nella coda:

| Esito | Soglia | Episodi con attività | Transizioni osservate | Già attiva al bordo / censura |
|---|---|---:|---:|---:|
| full | above_0_mm | 54 | 0 | 54 |
| full | above_10_mm | 53 | 7 | 46 |
| full | above_20_mm | 29 | 24 | 5 |
| full | above_30_mm | 0 | 0 | 0 |
| full | penalty_active | 29 | 24 | 5 |
| failed | above_0_mm | 32 | 18 | 14 |
| failed | above_10_mm | 32 | 25 | 7 |
| failed | above_20_mm | 32 | 32 | 0 |
| failed | above_30_mm | 4 | 4 | 0 |
| failed | penalty_active | 32 | 32 | 0 |

## Eventi vicini alla fine

Conteggi yes / no / unknown; “full” termina al limite temporale. Sono associazioni descrittive.

| Evento | Finestra [step] | full | failed |
|---|---:|---|---|
| resync | 5 | 1 / 53 / 0 | 0 / 32 / 0 |
| resync | 10 | 2 / 52 / 0 | 0 / 32 / 0 |
| resync | 25 | 6 / 48 / 0 | 0 / 32 / 0 |
| resync | 50 | 11 / 43 / 0 | 0 / 32 / 0 |
| hs_cancel | 5 | 0 / 54 / 0 | 0 / 32 / 0 |
| hs_cancel | 10 | 0 / 54 / 0 | 0 / 32 / 0 |
| hs_cancel | 25 | 0 / 54 / 0 | 0 / 32 / 0 |
| hs_cancel | 50 | 0 / 54 / 0 | 0 / 32 / 0 |
| policy_switch | 5 | 0 / 54 / 0 | 2 / 30 / 0 |
| policy_switch | 10 | 0 / 54 / 0 | 2 / 30 / 0 |
| policy_switch | 25 | 0 / 54 / 0 | 3 / 29 / 0 |
| policy_switch | 50 | 0 / 0 / 54 | 8 / 3 / 21 |

## Finestre della versione policy terminale

| Versioni pesi | Episodi | Completamento | Durata media [s] |
|---|---:|---:|---:|
| 70–74 | 18 | 66.7% | 4.301 |
| 75–79 | 48 | 56.2% | 4.581 |
| 80–84 | 20 | 75.0% | 4.698 |

| Versione durante l’episodio | Episodi | Completamento | Durata media [s] |
|---|---:|---:|---:|
| stable | 3 | 0.0% | 1.943 |
| switched | 83 | 65.1% | 4.644 |

Strati aggiuntivi in `summary.json`: versione iniziale/finale, coppia iniziale→finale, policy stabile/con cambi, durata×versione e qualità dei dati. Dettaglio episodio in `per_episode.jsonl`.

## Limiti

- Analisi descrittiva di episodi conclusi e scritti: episodi ancora aperti, processi morti o scritture perse non sono inclusi. La percentuale non coincide automaticamente con le metriche RLlib per iterazione.
- Il 38.8% storico è il contesto della diagnosi, non un denominatore di questa nuova telemetria. Confrontare finestre di policy ed esposizione prima di attribuire un miglioramento.
- Episodi lunghi hanno più opportunità di clipping, resync, cancellazioni e cambi di policy. Sono riportati tassi per 100 step, durata e strati per versione; questi confronti restano confusi dalla durata e dalla selezione degli episodi sopravvissuti.
- RMSE episodio è calcolato sugli step osservati dell’intero episodio; il RMSE tail riguarda al massimo gli ultimi 50. Gli episodi falliti coprono intervalli più brevi: gli RMSE non hanno supporto temporale identico.
- Le righe segment_completed=False sono escluse dal RMSE cinematico; il loro errore terminale resta assente, senza sostituirlo con il campione precedente. Nei record legacy senza questo flag il completamento del segmento resta sconosciuto e si conserva il conteggio storico. La durata e terminal_time_s usano il boundary di policy; terminal_contact_time_s e i picchi hard disponibili riportano separatamente il tempo misurato al substep.
- La precedenza temporale di resync, cancellazione o cambio policy non dimostra causalità. I conteggi entro N step confrontano la fine degli episodi falliti con la fine amministrativa dei completi; non stimano rischio per singolo evento.
- Onset indica la prima attività osservata nella coda, non necessariamente l’inizio dell’evento nell’intero episodio; attività già presente all’inizio è censurata a sinistra. Le soglie 0/10/20/30 mm sono diagnostiche; penalty_active usa la penalità effettivamente registrata.
- I cambi di policy temporizzati sono rilevati solo tra righe consecutive nella coda. 50 righe coprono 49 intervalli: l’assenza di cambi nei 50 step precedenti può restare unknown. Le versioni pesi non sono automaticamente numeri di iterazione.
- I record legacy privi del riepilogo policy_versions consentono solo controlli nella coda: first e switch_count dell’intero episodio restano sconosciuti. Nessuna ipotesi sul numero di worker.
