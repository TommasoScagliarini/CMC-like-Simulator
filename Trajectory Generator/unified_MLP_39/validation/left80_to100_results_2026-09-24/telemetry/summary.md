# Diagnosi offline degli episodi conclusi

Episodi letti: **179**; completamento: **58.7%**. Record duplicati: 0; righe invalide: 0.

| Esito | Episodi | Durata media [s] | Picco penetrazione medio [mm] | RMSE ginocchio [°] | RMSE caviglia [°] | Step con clipping |
|---|---:|---:|---:|---:|---:|---:|
| full | 105 | 5.000 | 23.259 | 7.751 | 4.018 | 0.0% |
| failed | 74 | 3.680 | 28.572 | 6.747 | 3.980 | 0.0% |

## Motivi e fasi terminali

- **full** — motivi: `{"episode_time_limit": 105}`; fasi: `{"SWING_AFTER_TO": 79, "STANCE_AFTER_HS": 26}`.
- **failed** — motivi: `{"grf_penetration": 69, "joint_divergence:pros_knee_angle": 2, "grf_penetration_hard": 2, "invalid_binary_event": 1}`; fasi: `{"STANCE_AFTER_HS": 69, "SWING_AFTER_TO": 5}`.

## Errori terminali e ultimi 50 step

| Esito | Errore finale ginocchio [°] | Errore finale caviglia [°] | RMSE tail ginocchio [°] | RMSE tail caviglia [°] |
|---|---:|---:|---:|---:|
| full | 12.588 | 4.300 | 11.093 | 5.050 |
| failed | 5.671 | 8.149 | 6.433 | 4.953 |

Errori terminali assoluti medi. Il dettaglio firmato è nel JSON. Prima attività di penetrazione nella coda:

| Esito | Soglia | Episodi con attività | Transizioni osservate | Già attiva al bordo / censura |
|---|---|---:|---:|---:|
| full | above_0_mm | 105 | 0 | 105 |
| full | above_10_mm | 102 | 16 | 86 |
| full | above_20_mm | 44 | 39 | 5 |
| full | above_30_mm | 0 | 0 | 0 |
| full | penalty_active | 44 | 39 | 5 |
| failed | above_0_mm | 74 | 35 | 39 |
| failed | above_10_mm | 74 | 62 | 12 |
| failed | above_20_mm | 73 | 73 | 0 |
| failed | above_30_mm | 2 | 2 | 0 |
| failed | penalty_active | 73 | 73 | 0 |

## Eventi vicini alla fine

Conteggi yes / no / unknown; “full” termina al limite temporale. Sono associazioni descrittive.

| Evento | Finestra [step] | full | failed |
|---|---:|---|---|
| resync | 5 | 6 / 99 / 0 | 4 / 70 / 0 |
| resync | 10 | 8 / 97 / 0 | 6 / 68 / 0 |
| resync | 25 | 18 / 87 / 0 | 8 / 66 / 0 |
| resync | 50 | 28 / 77 / 0 | 12 / 62 / 0 |
| hs_cancel | 5 | 0 / 105 / 0 | 0 / 74 / 0 |
| hs_cancel | 10 | 0 / 105 / 0 | 0 / 74 / 0 |
| hs_cancel | 25 | 0 / 105 / 0 | 0 / 74 / 0 |
| hs_cancel | 50 | 0 / 105 / 0 | 1 / 73 / 0 |
| policy_switch | 5 | 0 / 105 / 0 | 3 / 71 / 0 |
| policy_switch | 10 | 4 / 101 / 0 | 4 / 70 / 0 |
| policy_switch | 25 | 9 / 96 / 0 | 8 / 66 / 0 |
| policy_switch | 50 | 15 / 0 / 90 | 14 / 5 / 55 |

## Finestre della versione policy terminale

| Versioni pesi | Episodi | Completamento | Durata media [s] |
|---|---:|---:|---:|
| 80–84 | 39 | 66.7% | 4.534 |
| 85–89 | 48 | 58.3% | 4.392 |
| 90–94 | 43 | 58.1% | 4.479 |
| 95–99 | 49 | 53.1% | 4.429 |

| Versione durante l’episodio | Episodi | Completamento | Durata media [s] |
|---|---:|---:|---:|
| stable | 5 | 0.0% | 2.426 |
| switched | 174 | 60.3% | 4.512 |

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
