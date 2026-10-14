# Verifica indipendente degli output salvati — 2026-10-13

Instruction check token: CMC_AGENT_OK_2026

Casi selezionati: 21; non finalizzati: 7. Harness: `7a218968860ce58977f6b112fae6e105faff787f04c2a89d5340f9cda4d6aee5`.

Nessun output Claude13 o conclusione numerica root letti. Solo artifact finalizzati; nessun import/test nativo o modifica agli input congelati.

| Caso | Status receipt | Fine summary | Step restituiti | STO | Finitezza | Bio ref identica | Reward somma | Plot |
|---|---|---|---:|---:|---|---|---|---:|
| coherent_nominal | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_003 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_004 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_005 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_006 | rollout_complete | morphology_causal_contract_failure | 32 | 17/17 | True | True | True | 7/7 |
| random_007 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_008 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_009 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_010 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_013 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_014 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_015 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_016 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_017 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_018 | non finalizzato | — | — | — | — | — | — | — |
| random_019 | non finalizzato | — | — | — | — | — | — | — |
| random_020 | non finalizzato | — | — | — | — | — | — | — |
| random_021 | non finalizzato | — | — | — | — | — | — | — |
| random_022 | non finalizzato | — | — | — | — | — | — | — |
| random_023 | non finalizzato | — | — | — | — | — | — | — |
| random_025 | non finalizzato | — | — | — | — | — | — | — |

I bio reference hash riguardano soltanto time+colonne biologiche q/qdot, con token salvati e prefisso comune; le reference protesiche possono differire. Stati fisici effettivi e reference sono controlli distinti.

La forza e verificata dalla norma del vettore APPLIED sinistro e dal mass/guard effettivo, non dalla sola normal force. Clamp motore riportato soltanto quando il canale tau_input_plugin reale e disponibile e coerente col flag; u saturato e un altro fenomeno.

Step/return vengono ricostruiti dai reward realmente restituiti. I casi censurati o nativamente interrotti non ricevono reward o campioni ipotetici della transizione fallita. Completamento temporale, ciclo HS→TO→HS e validita biomeccanica non sono dedotti dal solo stato degli artifact.

Evidenze complete: [independent_saved_output_verification_partial.json](independent_saved_output_verification_partial.json).
