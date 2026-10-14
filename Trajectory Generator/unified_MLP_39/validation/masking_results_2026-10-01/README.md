# Rollout del training con oscuramento — 2026-10-01

Selezione per massimo episode_return_mean del training: CP112 tra le iterazioni 91–190; CP184 tra le ultime 25 (166–190). CP90 resta la baseline dichiarata.

Entrambi i rollout sono deterministici: seed 123, start offset 1 s, durata 5 s, 500 step, disponibilità target 0 (oscuramento completo).

| Checkpoint | Ritorno training | Ritorno rollout | Cicli validi | Eventi invalidi | Penetrazione max [mm] |
|---|---:|---:|---:|---:|---:|
| CP112 | 249.325 | 138.191 | 2 | 0 | 17.632 |
| CP184 | 158.711 | 210.421 | 2 | 0 | 19.181 |

Entrambi completano la finestra (episode_time_limit), senza terminazione anticipata. CP184 ottiene un ritorno maggiore in questo singolo confronto a oscuramento completo; i ritorni del training provengono da regimi di visibilità diversi. La figura 7 mostra ancora scostamenti dal target, soprattutto nei picchi del ginocchio.

Verifiche: sei ingressi prescribed nulli su tutti i 500 step di ciascun rollout; checkpoint sorgente invariati; sette PNG validi per checkpoint; nessun canale mancante. Ispezionata visivamente la figura 7 di entrambi.

## Artefatti

- CP112: plot in `/Users/tommy/Documents/CMC-like-Simulator - Claude/plot/10_01_2026_unified39_LEFT_masking_best112_overall_hidden`; rollout in `/Users/tommy/Documents/CMC-like-Simulator - Claude/Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_LEFT_masking_best112_overall_hidden_2026-10-01`.
- CP184: plot in `/Users/tommy/Documents/CMC-like-Simulator - Claude/plot/10_01_2026_unified39_LEFT_masking_best184_last25_hidden`; rollout in `/Users/tommy/Documents/CMC-like-Simulator - Claude/Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_LEFT_masking_best184_last25_hidden_2026-10-01`.

Provenienza e hash: `selection.json`; risultati e verifiche: `results.json`; comandi ed esiti: `best112_execution.json` e `best184_execution.json`. Ogni cartella plot contiene anche `plot_receipt.json` e `verification.json`.

TODO preesistente: bug PPO bootstrap Ray 2.55.1 rinviato su indicazione dell’utente, non modificato in questa attività.
