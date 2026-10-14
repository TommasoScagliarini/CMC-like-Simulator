# Verifica indipendente degli output salvati — 2026-10-13

Instruction check token: CMC_AGENT_OK_2026

Casi selezionati: 21; non finalizzati: 0. Harness: `7a218968860ce58977f6b112fae6e105faff787f04c2a89d5340f9cda4d6aee5`.

Non sono stati letti output Claude13 o report/sintesi numeriche root della campagna. Le verifiche sono ricostruite dai raw finalizzati; nessun import/test nativo o modifica agli input congelati.

Nota sulla provenienza: dopo il primo draft del validator il parent ha inviato un esempio operativo relativo al terminale di random_006. Tale esposizione e dichiarata nel JSON; i file raw sono stati verificati direttamente e non sono stati letti output Claude o sintesi numeriche della campagna.

Verificati 357 STO (357 finiti), 147 PNG con hash/percorso corretti e 76/76 input protetti invariati. Bootstrap coerente in 21 casi e byte della reference biologica identici su prefissi comuni in 21 casi.

Questi conteggi misurano l'integrita degli artifact. Lo status `rollout_complete` significa ritorno normale dell'evaluator; il completamento5s viene verificato da `completed_requested_duration`, e la sequenza HS→TO→HS costituisce un criterio funzionale separato.

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
| random_018 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_019 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_020 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_021 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_022 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_023 | rollout_complete | episode_time_limit | 500 | 17/17 | True | True | True | 7/7 |
| random_025 | rollout_complete | joint_divergence:pros_knee_angle | 14 | 17/17 | True | True | True | 7/7 |

Il caso random_025 viene identificato dai dati raw come terminazione `joint_divergence:pros_knee_angle`, coerente fra summary e ultimo reward journal. L'ultimo q knee e 0.0004368696902059392 rad, oltre il limite operativo superiore0 rad (bounds [-2.0, 0.0]), al tempo 14.086870983805024 s. Ha 14 reward restituiti e 14 finestre complete; l'uscita dal limite operativo non equivale a uscita dal range ±pi/2 dichiarato nel modello.

Witness misurato del guard fisico di penetrazione nei journal: random_006. Il timestamp fisico e distinto dal logical policy time; l'etichetta riportata dal wrapper nella summary resta conservata senza cancellare il guard sottostante.

I bio reference hash riguardano soltanto time+colonne biologiche q/qdot, con token salvati e prefisso comune; le reference protesiche possono differire. Stati fisici effettivi e reference sono controlli distinti.

La forza e verificata dalla norma del vettore APPLIED sinistro e dal mass/guard effettivo, non dalla sola normal force. Clamp motore riportato soltanto quando il canale tau_input_plugin reale e disponibile e coerente col flag; u saturato e un altro fenomeno.

Step/return vengono ricostruiti dai reward realmente restituiti. I casi censurati o nativamente interrotti non ricevono reward o campioni ipotetici della transizione fallita. Completamento temporale, ciclo HS→TO→HS e validita biomeccanica non sono dedotti dal solo stato degli artifact.

Evidenze complete: [independent_saved_output_verification.json](independent_saved_output_verification.json).
