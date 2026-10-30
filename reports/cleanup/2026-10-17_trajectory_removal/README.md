# Pulizia di Trajectory Generator — 2026-10-17

Sono state eliminate, su richiesta dell’utente, le due varianti deprecate
`unified_MLP/` e `unified_MLP_39/` e la cartella degli esperimenti
`Trajectory Generator/validation/`. Sono compresi sorgenti, checkpoint,
configurazioni, immagini e risultati contenuti nelle tre cartelle.

| Cartella rimossa | File | Dimensione |
| --- | ---: | ---: |
| `Trajectory Generator/unified_MLP/` | 12,524 | 2.53 GB |
| `Trajectory Generator/unified_MLP_39/` | 39,629 | 11.80 GB |
| `Trajectory Generator/validation/` | 1,744 | 1.64 GB |

I collegamenti nei report che puntavano a questi materiali ora rimandano
alla relativa voce di questo registro, con la dicitura «materiale rimosso».
Queste voci documentano la rimozione: non contengono gli artefatti originali.
Quando esiste una copia identica conservata, il collegamento punta a quella.

L’utente ha dichiarato di avere copiato l’intera repository su un disco
esterno prima della pulizia. Per cercare un artefatto in quella copia,
usare il percorso originale riportato sotto; il percorso del disco esterno
non è stato fornito e non è verificato da questo registro.

Le analisi nei report restano riferite allo stato storico del progetto.
I percorsi e gli hash nei manifest, nei checkpoint e nelle ricevute storiche
conservate mantengono il loro significato originale.

Il codice operativo `baseline_MLP/`, i suoi esperimenti, i training in
`Trajectory Generator/runs/`, il checkpoint H0 nella `validation/` del root,
i moduli condivisi, i modelli e i plugin restano fuori dalla rimozione.

La [mappa dei collegamenti](report_links.json) conserva i destinatari originali.
La [verifica finale](verification.json) riporta i controlli eseguiti.

## Registro dei materiali citati nei report

<a id="artifact-47553b1e2e1630c0"></a>

### README.md

Percorso originale: `Trajectory Generator/unified_MLP/README.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-4bfd4583c8d75f29"></a>

### TODO.md

Percorso originale: `Trajectory Generator/unified_MLP/TODO.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-9c819e86ce3077bd"></a>

### VALIDATION.md

Percorso originale: `Trajectory Generator/unified_MLP/VALIDATION.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-adcbbd6627206698"></a>

### provenance.json

Percorso originale: `Trajectory Generator/unified_MLP/configs/provenance.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-352d13a3c98b2b9b"></a>

### env.py

Percorso originale: `Trajectory Generator/unified_MLP/env.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-ae98c46bec812946"></a>

### env_factory.py

Percorso originale: `Trajectory Generator/unified_MLP/env_factory.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-fcc395cfa8e7cd1d"></a>

### STATUS.md

Percorso originale: `Trajectory Generator/unified_MLP/runs/plans/pretraining_50_100_feasibility_2026-09-14/STATUS.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-9c10c379d81c6d8d"></a>

### evaluation_50_recovery.json

Percorso originale: `Trajectory Generator/unified_MLP/runs/plans/pretraining_50_100_feasibility_2026-09-14/evaluation_50_recovery.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-e2782b0cfd3a5e2a"></a>

### preflight.json

Percorso originale: `Trajectory Generator/unified_MLP/runs/plans/pretraining_50_100_feasibility_2026-09-14/preflight.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-1cb076b1bcfc2595"></a>

### training_summary_50.json

Percorso originale: `Trajectory Generator/unified_MLP/runs/plans/pretraining_50_100_feasibility_2026-09-14/training_summary_50.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-eb1df1877ee4e5c8"></a>

### STATUS.md

Percorso originale: `Trajectory Generator/unified_MLP/runs/plans/pretraining_cp200_to300_2026-09-16/STATUS.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-d03731218c164bd2"></a>

### completion_verification.json

Percorso originale: `Trajectory Generator/unified_MLP/runs/plans/pretraining_cp200_to300_2026-09-16/completion_verification.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-f9e8f0ee7d865feb"></a>

### summary.json

Percorso originale: `Trajectory Generator/unified_MLP/runs/training/pretraining_v1_pen1_pilot_2026-09-14/summary.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-85a2f7579e23a2c1"></a>

### pretraining_v1_continuation_300_2026-09-17

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

Il destinatario era già assente alla verifica precedente alla rimozione.

<a id="artifact-fec8ddf5bd0fef19"></a>

### deterministic_comparison_200_300.pdf

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/deterministic_comparison_200_300.pdf`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-f4453579acc840bd"></a>

### deterministic_comparison_200_300.png

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/deterministic_comparison_200_300.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-2ac90c61d2b4fa77"></a>

### evaluation_diagnostics.json

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/evaluation_diagnostics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-6f3186f1844c54ea"></a>

### evaluation_summary.csv

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/evaluation_summary.csv`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-222bf2ceb4852a5e"></a>

### kinematics_200_283_300.pdf

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/kinematics_200_283_300.pdf`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-e76714c032520a41"></a>

### kinematics_200_283_300.png

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/kinematics_200_283_300.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-61a2ec2f0dbc72ec"></a>

### milestone_evaluation.json

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/milestone_evaluation.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-b104f38739a3234c"></a>

### milestone_evaluation.md

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/milestone_evaluation.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-19b1f49ccadb5f6b"></a>

### runtime_and_metric_audit.md

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/runtime_and_metric_audit.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-53bf77dd645638c0"></a>

### runtime_changes_after_launch.diff

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/runtime_changes_after_launch.diff`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-d0dc10d0a6ce4aeb"></a>

### training_181_300.png

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/training_181_300.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-f410692eb960eab5"></a>

### training_1_300.pdf

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/training_1_300.pdf`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-0305c8013ae49f10"></a>

### training_1_300.png

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/training_1_300.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-af4834c7d12e77e4"></a>

### training_analysis.json

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/training_analysis.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-500b9f5341550a25"></a>

### training_compute.py

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/training_compute.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-664ccbe6184e6450"></a>

### training_series.csv

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_continuation_300_2026-09-17/training_series.csv`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-9bdc8325befa7efc"></a>

### comparison.md

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_compare_3_5_2026-09-14/comparison.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-1cb3373b67e47e1b"></a>

### completion_verification.json

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_compare_3_5_2026-09-14/completion_verification.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-fc062346f40de0f9"></a>

### tracking_angles.png

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_compare_3_5_2026-09-14/tracking_angles.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-aa3f1404df94c719"></a>

### metrics.csv

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/metrics.csv`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-0b66075f05f039a9"></a>

### milestone_evaluation.json

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/milestone_evaluation.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-89668af746e67637"></a>

### milestone_evaluation.md

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/milestone_evaluation.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-f29e116033e59b2b"></a>

### tracking_center.pdf

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/tracking_center.pdf`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-090747e54fde5d71"></a>

### tracking_center.png

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/tracking_center.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-87bbf97ae45e314b"></a>

### tracking_early.pdf

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/tracking_early.pdf`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-a3ccb38b8370565b"></a>

### tracking_early.png

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/tracking_early.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-a979666bba519448"></a>

### tracking_late.pdf

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/tracking_late.pdf`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-6324d0f8cb0ca9d7"></a>

### tracking_late.png

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v1_pen1_milestone_50_2026-09-14/tracking_late.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-3fdb999918e4cb3f"></a>

### comparison_eta.md

Percorso originale: `Trajectory Generator/unified_MLP/runs/validation/pretraining_v26_comparison_eta_2026-09-14/comparison_eta.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-d85132b1ee900779"></a>

### analyze_training_300.py

Percorso originale: `Trajectory Generator/unified_MLP/validation/analyze_training_300.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-589c572b818f0654"></a>

### evaluate_training_300.py

Percorso originale: `Trajectory Generator/unified_MLP/validation/evaluate_training_300.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-ac566364fb3aec4d"></a>

### evaluate_training_milestone.py

Percorso originale: `Trajectory Generator/unified_MLP/validation/evaluate_training_milestone.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-e433eb332c7ca631"></a>

### build_variant.py

Percorso originale: `Trajectory Generator/unified_MLP/validation/feasibility39_bundle/build_variant.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-0e194cbc0db6d4e6"></a>

### run_authorized_plan.py

Percorso originale: `Trajectory Generator/unified_MLP/validation/run_authorized_plan.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-f221263086aea844"></a>

### asymmetric_rl_module.py

Percorso originale: `Trajectory Generator/unified_MLP_39/asymmetric_rl_module.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-a71f16657df74cfd"></a>

### ex_novo.yaml

Percorso originale: `Trajectory Generator/unified_MLP_39/configs/ex_novo.yaml`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-081f13b1df694839"></a>

### left_fresh_best80config_100iter_2026-09-29.yaml

Percorso originale: `Trajectory Generator/unified_MLP_39/configs/left_fresh_best80config_100iter_2026-09-29.yaml`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-d8df9f1480de598e"></a>

### reward_tuning_left.yaml

Percorso originale: `Trajectory Generator/unified_MLP_39/configs/reward_tuning_left.yaml`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-9f6a9111962438c5"></a>

### v26_aligned_right_phase_fsmv3_40iter_2026-09-23.yaml

Percorso originale: `Trajectory Generator/unified_MLP_39/configs/v26_aligned_right_phase_fsmv3_40iter_2026-09-23.yaml`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-0fe045e94030c646"></a>

### README.md

Percorso originale: `Trajectory Generator/unified_MLP_39/reward_tuning/README.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-497ad289bd46c16a"></a>

### noise_analysis.md

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/completion_recovery42_2026-09-22/analysis_iter060/noise_analysis.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-46df9dc9bb280b63"></a>

### comparison.md

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/completion_recovery42_2026-09-22/comparison.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-f2f57dca18f87050"></a>

### summary.md

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/completion_recovery42_2026-09-22/telemetry_analysis_iter060/summary.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-8ec4e895ce608a8c"></a>

### comparison.md

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/evaluation_best72_2026-09-22/comparison.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-79357e1c36826f1c"></a>

### README.md

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/left_best90_masking_100iter_2026-09-30/README.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-c855f3acc71e824f"></a>

### progress.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/left_best90_masking_100iter_2026-09-30/progress.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-b5d44d81c7824dc8"></a>

### left_fresh_best80config_100iter_2026-09-29

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/left_fresh_best80config_100iter_2026-09-29`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

Il destinatario era già assente alla verifica precedente alla rimozione.

<a id="artifact-8cb27a2c2ce1b492"></a>

### README.md

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/phase_clock_ablation_2026-09-18/README.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-4863a74d249c1d05"></a>

### preflight.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/phase_clock_ablation_2026-09-18/preflight.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-573af4f46b0f221c"></a>

### results.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/phase_clock_ablation_2026-09-18/results.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-bab82e1793b045de"></a>

### config_40_to_60.yaml

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/right_reference_resume60_2026-09-23/config_40_to_60.yaml`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-03d11e7be18281db"></a>

### status.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/right_reference_resume60_2026-09-23/status.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-73fd8e950029c7ea"></a>

### protocol.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/right_reference_training_2026-09-23/protocol.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-6df43ed8be9c8cbd"></a>

### status.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/right_reference_training_2026-09-23/status.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-671e67c426343bc5"></a>

### training.log

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/right_reference_training_2026-09-23/training.log`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-06e61a6e2e8d833e"></a>

### config.yaml

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/plans/v26_aligned_left_phase_fsmv3_25iter_2026-09-21/config.yaml`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-a4272d7dceee8afc"></a>

### analysis.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_best80_2026-09-25/analysis/analysis.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-c465d3ef20ee64b5"></a>

### report.md

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_best80_2026-09-25/analysis/report.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-0e649f6ca765eec2"></a>

### comparison.md

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_best80_2026-09-25/comparison.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-4d8fa938c0d7c490"></a>

### launch_receipt.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_best80_2026-09-25/pilot/launch_receipt.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-d5dc8ff8273f953b"></a>

### pilot_manifest.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_best80_2026-09-25/pilot/pilot_manifest.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-f4254909c9468124"></a>

### pilot_state.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_best80_2026-09-25/pilot/pilot_state.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-9398c2c6609a9f32"></a>

### startup_check.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_best80_2026-09-25/pilot/startup_check.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-3235408ad8f801b2"></a>

### verification.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_best80_2026-09-25/verification.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-2f540a74e927820b"></a>

### report.md

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_best80_ankle5_2026-09-25/report.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-dfe245037d862b6f"></a>

### report.md

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_pilot_diagnosis_2026-09-28/analysis/report.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-74e3d7c45e2ba609"></a>

### report.md

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_pilot_diagnosis_2026-09-28/phase_analysis/report.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-587de19e52f59be2"></a>

### report.md

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_pilot_diagnosis_2026-09-28/report.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-d78466abe2cef5ec"></a>

### report.md

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_pilot_diagnosis_2026-09-28/training_analysis/report.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-1bc73cf104bd3df0"></a>

### verification.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/reward_tuning/left_pilot_diagnosis_2026-09-28/verification.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-5ca9412348973bdd"></a>

### MLP39_LEFT_masking_best112_overall_hidden_2026-10-01

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_LEFT_masking_best112_overall_hidden_2026-10-01`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

Il destinatario era già assente alla verifica precedente alla rimozione.

<a id="artifact-f4c878ae727c60e3"></a>

### MLP39_LEFT_masking_best184_last25_hidden_2026-10-01

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_LEFT_masking_best184_last25_hidden_2026-10-01`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

Il destinatario era già assente alla verifica precedente alla rimozione.

<a id="artifact-5d14b065b9c6dfbf"></a>

### 09_19_2026_1

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_phase_baseline_phase_2026-09-18/center/plots/09_19_2026_1`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

Il destinatario era già assente alla verifica precedente alla rimozione.

<a id="artifact-7c8d3fb9bc57d288"></a>

### 07_mlp_policy_vs_sound_leg_error.png

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_phase_baseline_phase_2026-09-18/center/plots/09_19_2026_1/07_mlp_policy_vs_sound_leg_error.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-90fc39664d4edc6b"></a>

### 09_20_2026_1

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_phase_no_slew_2026-09-18/center/plots/09_20_2026_1`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

Il destinatario era già assente alla verifica precedente alla rimozione.

<a id="artifact-7590f96fe0db0a50"></a>

### 07_mlp_policy_vs_sound_leg_error.png

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_phase_no_slew_2026-09-18/center/plots/09_20_2026_1/07_mlp_policy_vs_sound_leg_error.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-86beb8a04da72d92"></a>

### 09_19_2026_1

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_phase_velocity_scale1_2026-09-18/center/plots/09_19_2026_1`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

Il destinatario era già assente alla verifica precedente alla rimozione.

<a id="artifact-08c07681e8de4c0e"></a>

### 07_mlp_policy_vs_sound_leg_error.png

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_phase_velocity_scale1_2026-09-18/center/plots/09_19_2026_1/07_mlp_policy_vs_sound_leg_error.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-4c15a64ef464c6df"></a>

### 01_time_sea_control_reserve.png

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25/plots/09_21_2026_1/01_time_sea_control_reserve.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-4d1a065b46f68ab6"></a>

### 02_time_joint_motor_states.png

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25/plots/09_21_2026_1/02_time_joint_motor_states.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-8b1f1a284f386603"></a>

### 03_gaitcycle_torque_angle_power.png

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25/plots/09_21_2026_1/03_gaitcycle_torque_angle_power.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-c03eb8cd7af55629"></a>

### 04_gaitcycle_joint_velocity_power.png

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25/plots/09_21_2026_1/04_gaitcycle_joint_velocity_power.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-604d88e751bad8ba"></a>

### 05_time_tau_input_tracking_error.png

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25/plots/09_21_2026_1/05_time_tau_input_tracking_error.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-4cbbe920deb868c0"></a>

### 06_time_joint_ref_sea_error.png

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25/plots/09_21_2026_1/06_time_joint_ref_sea_error.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-a18cbfd59dad4bd4"></a>

### 07_mlp_policy_vs_sound_leg_error.png

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25/plots/09_21_2026_1/07_mlp_policy_vs_sound_leg_error.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-225434faef44cfcd"></a>

### rollout_summary.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21_best25/rollout_summary.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-5e92362ab277ebbc"></a>

### plot_receipt.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_best29/plots/plot_receipt.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-ef86be496b4e7b4a"></a>

### rollout_summary.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_best29/rollout_summary.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-d346582769261e32"></a>

### plot_receipt.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40/plots/plot_receipt.json`.

È disponibile una [copia identica conservata](<../../../plot/09_21_2026_1_unified39_LEFT_iter40/plot_receipt.json>).

<a id="artifact-149139f115e1a9a3"></a>

### rollout_summary.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/MLP39_v26_aligned_left_phase_fsmv3_40iter_2026-09-21_last40/rollout_summary.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-a922b5bf17052f05"></a>

### V26_best87_in_unified39_right_fsmv3_2026-09-23

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/V26_best87_in_unified39_right_fsmv3_2026-09-23`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

Il destinatario era già assente alla verifica precedente alla rimozione.

<a id="artifact-31d9f808898a9bdc"></a>

### rollout_summary.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/causal_difference_2026-09-21_oracle_left/rollout_summary.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-6a5335ea7e3893b0"></a>

### rollout_summary.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/causal_difference_2026-09-21_oracle_right/rollout_summary.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-db376ebb13abed03"></a>

### rollout_summary.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/rollout/unified_MLP_39/runs/rollout/MLP39_LEFT_fresh_best90_2026-09-30_deterministic/rollout_summary.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-8b79ed9dccd92cfa"></a>

### MLP39_LEFT_fresh_best80config_100iter_2026-09-29

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/training/MLP39_LEFT_fresh_best80config_100iter_2026-09-29`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

Il destinatario era già assente alla verifica precedente alla rimozione.

<a id="artifact-07181fe70ed86880"></a>

### summary.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/training/MLP39_LEFT_fresh_best80config_100iter_2026-09-29/summary.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-c969496862b0aacf"></a>

### summary.json

Percorso originale: `Trajectory Generator/unified_MLP_39/runs/training/MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21/summary.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-ba00f840c70b0711"></a>

### target_availability.py

Percorso originale: `Trajectory Generator/unified_MLP_39/target_availability.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-fad6b014887afaa8"></a>

### 2026-09-17_left_phase_implementation.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/2026-09-17_left_phase_implementation.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-939e75ce4ab7dd4d"></a>

### analyze_results.py

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/analyze_results.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-33841a104c548ce1"></a>

### ankle_all_starts.png

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/ankle_all_starts.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-31d34f15d3373398"></a>

### historical_comparison.png

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/historical_comparison.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-edbdf1d57a690882"></a>

### input_hashes.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/input_hashes.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-5bcb538087b8f5c3"></a>

### knee_all_starts.png

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/knee_all_starts.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-2bea099b073a735f"></a>

### metrics.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/metrics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-909354628e183518"></a>

### report_verification.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/report_verification.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-6fbd080b02898f9e"></a>

### training_curves.png

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/training_curves.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-710710d9e0d2cad8"></a>

### write_report.py

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/ablation_report_2026-09-20/write_report.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-2eec0f16388a418a"></a>

### best80_torque_audit_2026-09-24

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/best80_torque_audit_2026-09-24`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

Il destinatario era già assente alla verifica precedente alla rimozione.

<a id="artifact-5af93fe8626586a7"></a>

### analyze_oracles.py

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/analyze_oracles.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-24bc17830db62a4e"></a>

### findings.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/fsm_audit/findings.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-1fb00c998ba9e7fc"></a>

### loss_reconstruction.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/loss_reconstruction.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-5089e64453631632"></a>

### oracle_comparison.png

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/oracle_comparison.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-23f4969f47afd4a3"></a>

### oracle_metrics.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/oracle_metrics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-66115f86ff779fbf"></a>

### oracle_probe.py

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/oracle_probe.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-3d7fba527e8b8326"></a>

### README.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/target_audit/README.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-045bff871c8b9570"></a>

### target_contact_comparison.png

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/target_audit/target_contact_comparison.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-d88fc18f9909098a"></a>

### findings.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/training_audit/findings.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-aea128497b9f6af9"></a>

### physical_provenance.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/causal_difference_2026-09-21/training_audit/physical_provenance.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-7f016a52c9a3685f"></a>

### diagnosi.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/diagnosis_v26_2026-09-18/diagnosi.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-48a90259538f0d09"></a>

### final_metrics.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/final_completion_analysis_2026-09-22/final_metrics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-a8802777889331ef"></a>

### noise_audit.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/final_completion_analysis_2026-09-22/noise_audit.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-38baf8345b574576"></a>

### PROTOCOL.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/general_method_diagnosis_2026-09-29/PROTOCOL.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-f91994e37ae150ae"></a>

### README.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/general_method_diagnosis_2026-09-29/dynamic/README.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-72d96d3e41c9e7b1"></a>

### final_verification.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/general_method_diagnosis_2026-09-29/final_verification.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-e5fc4507110fe2ce"></a>

### README.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/general_method_diagnosis_2026-09-29/quality/README.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-8f8139bfa0fa7c37"></a>

### hard_guard_contact_2026-09-24

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/hard_guard_contact_2026-09-24`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

Il destinatario era già assente alla verifica precedente alla rimozione.

<a id="artifact-32cc5824817fd557"></a>

### live_smoke.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/hard_guard_contact_2026-09-24/live_smoke.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-3fb9b5d3ac66bb4f"></a>

### left_reward_audit_2026-09-24

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/left_reward_audit_2026-09-24`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

Il destinatario era già assente alla verifica precedente alla rimozione.

<a id="artifact-3b5e3afe30d1a721"></a>

### candidate_penetration_weight_2.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/left_reward_audit_2026-09-24/candidate_penetration_weight_2.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-ed02cbcefbb9a2a1"></a>

### results.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/masking_results_2026-10-01/results.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-76f04ca86d294ee1"></a>

### selection.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/masking_results_2026-10-01/selection.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-6dda9a1696b284ae"></a>

### phase_delay_torque_2026-09-22

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/phase_delay_torque_2026-09-22`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

Il destinatario era già assente alla verifica precedente alla rimozione.

<a id="artifact-301a2a4ef7f31811"></a>

### alignment_diagnostic.png

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/phase_delay_torque_2026-09-22/lag_audit/alignment_diagnostic.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-efab278d5fecf67f"></a>

### last_stance_cascade.png

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/phase_delay_torque_2026-09-22/torque_audit/last_stance_cascade.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-5c3e8c3d20433343"></a>

### PROTOCOL.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/reward_curriculum_2026_10_01/PROTOCOL.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-e7f022fd4a935ff5"></a>

### README.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/reward_curriculum_2026_10_01/README.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-2ede5a0f72d744f1"></a>

### verification.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/reward_curriculum_2026_10_01/verification.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-dd880b41e4e75ad8"></a>

### reward_tuning_smoke_2026-09-25.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/reward_tuning_smoke_2026-09-25.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-e4e190e82b08736b"></a>

### README.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/right_reference_results60_2026-09-23/README.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-6ddffe966fc83eb5"></a>

### findings.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/right_reference_results60_2026-09-23/failure_seed125/findings.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-e62fc8b7eac3ccdc"></a>

### metrics.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/right_reference_results60_2026-09-23/noise_screening/metrics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-0ce283ac460b5333"></a>

### README.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/right_reference_results_2026-09-23/README.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-d2d6f2786dd42165"></a>

### final_verification.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/right_reference_results_2026-09-23/final_verification.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-153be6fd9838f005"></a>

### metrics.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/right_reference_results_2026-09-23/fsm_audit/metrics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-a293157cbfbd80f3"></a>

### rollout_metrics.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/right_reference_results_2026-09-23/rollout_metrics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-df1e7269c52ee12a"></a>

### findings.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/right_reference_results_2026-09-23/training_history/findings.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-894f4299d2bc5af0"></a>

### README.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/right_reference_resume60_2026-09-23/README.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-e168bf9a96219724"></a>

### README.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/right_reference_screening_2026-09-23/README.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-df4df5c4c496d999"></a>

### metrics.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/right_reference_screening_2026-09-23/metrics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-4b99a28f2b144ccd"></a>

### historical_stack.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/rl_learning_audit_2026-09-29/historical_stack.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-cf7ae05d793f1630"></a>

### report.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/rl_learning_audit_2026-09-29/replay_comparison/report.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-566342d48456be9f"></a>

### verification.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/rl_learning_audit_2026-09-29/verification.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-89046076f6d419c5"></a>

### analyze.py

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_aligned_25iter_2026-09-21/analyze.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-5fc61642770cf868"></a>

### metrics.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_aligned_25iter_2026-09-21/metrics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-94a694cc917347a7"></a>

### rollout_comparison.png

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_aligned_25iter_2026-09-21/rollout_comparison.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-731f4410204232a1"></a>

### sampling_budget_audit.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_aligned_25iter_2026-09-21/sampling_budget_audit.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-6912e4c36c25f10b"></a>

### training_comparison.png

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_aligned_25iter_2026-09-21/training_comparison.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-d0eb97c9d9b3a10d"></a>

### analyze.py

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_aligned_40iter_2026-09-21/analyze.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-d34c6a8c532e21c8"></a>

### contact_moment_comparison.png

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_aligned_40iter_2026-09-21/contact_moment_comparison.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-d79c21ac404e6616"></a>

### metrics.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_aligned_40iter_2026-09-21/metrics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-554c0140314a0b1c"></a>

### rollout_comparison.png

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_aligned_40iter_2026-09-21/rollout_comparison.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-ff89b36068f9bc83"></a>

### training_by_samples.png

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_aligned_40iter_2026-09-21/training_by_samples.png`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-c0fe5f8648e7cd34"></a>

### README.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_robustness_2026-09-23/README.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-752c905a6e23f095"></a>

### all_metrics.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_robustness_2026-09-23/all_metrics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-1c907e94ec1cde22"></a>

### deterministic_sto_equality.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_robustness_2026-09-23/deterministic_sto_equality.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-7eef8d8dd18b5124"></a>

### final_verification.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_robustness_2026-09-23/final_verification.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-8394c096ec7dd6b7"></a>

### README.md

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_weight_transfer_2026-09-23/README.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-07d03d07c3dda63f"></a>

### analyze.py

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_weight_transfer_2026-09-23/analyze.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-15b6a9fdfde0c2f4"></a>

### artifact_verification.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_weight_transfer_2026-09-23/artifact_verification.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-1ebcbda9eabdecfc"></a>

### metrics.json

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_weight_transfer_2026-09-23/metrics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-560d4a9edc315066"></a>

### probe.py

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_weight_transfer_2026-09-23/probe.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-aa85364c2ac49f66"></a>

### verify_artifacts.py

Percorso originale: `Trajectory Generator/unified_MLP_39/validation/v26_weight_transfer_2026-09-23/verify_artifacts.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-0036375171d9a8fc"></a>

### result_summary.json

Percorso originale: `Trajectory Generator/validation/AB06/measured_divergence_2026_10_06/result_summary.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-39c61ea830c9e497"></a>

### result_summary.json

Percorso originale: `Trajectory Generator/validation/AB06/v26c_best47_control_2026_10_06/result_summary.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-3866d980498c9741"></a>

### AB07

Percorso originale: `Trajectory Generator/validation/AB07`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

Il destinatario era già assente alla verifica precedente alla rimozione.

<a id="artifact-509d692b5c9043a0"></a>

### analyze_prescribed_v26.py

Percorso originale: `Trajectory Generator/validation/AB07/analyze_prescribed_v26.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-8850f6eb16146587"></a>

### result_summary.json

Percorso originale: `Trajectory Generator/validation/AB07/prescribed_ground_registered_2026_10_06/result_summary.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-0f5eb35e62d4d4ea"></a>

### qualify_prescribed_v26.py

Percorso originale: `Trajectory Generator/validation/AB07/qualify_prescribed_v26.py`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-7ca6be26832954af"></a>

### metrics.json

Percorso originale: `Trajectory Generator/validation/AB07/v26c_best47_pylon400_2026_10_07/analysis/metrics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-5ddc06c9ce190796"></a>

### phase_metrics.json

Percorso originale: `Trajectory Generator/validation/AB07/v26c_best47_pylon400_2026_10_07/analysis/phase_metrics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-866744d29e66bfb1"></a>

### analysis_timing_qualification.json

Percorso originale: `Trajectory Generator/validation/AB07/v26c_best47_pylon400_2026_10_07/analysis_timing_qualification.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-a9b821f2320e46a6"></a>

### final_completion_receipt.json

Percorso originale: `Trajectory Generator/validation/AB07/v26c_best47_pylon400_2026_10_07/final_completion_receipt.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-7970d6a0b5ca9305"></a>

### learned_vs_CMC_contract_review.json

Percorso originale: `Trajectory Generator/validation/AB07/v26c_best47_pylon400_2026_10_07/learned_vs_CMC_contract_review.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-cef5f7cde5b9030c"></a>

### result_summary.txt

Percorso originale: `Trajectory Generator/validation/AB07/v26c_best47_pylon400_2026_10_07/result_summary.txt`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-047412ee2e9d7fe0"></a>

### result_summary.json

Percorso originale: `Trajectory Generator/validation/AB07/v26c_best47_transfer_2026_10_06/result_summary.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-acc38d640b404a89"></a>

### numerical_verification.json

Percorso originale: `Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07/AB06/numerical_verification.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-e06d5baced778202"></a>

### numerical_verification.json

Percorso originale: `Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07/AB07/numerical_verification.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-f5f2644fc62adf6c"></a>

### result_summary.json

Percorso originale: `Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07/result_summary.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-4cadbcfbce5304f4"></a>

### selection.json

Percorso originale: `Trajectory Generator/validation/paired_prescribed_AB06_AB07_2026_10_07/selection.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-610d04754a4291c7"></a>

### aggregate_results.json

Percorso originale: `Trajectory Generator/validation/random_pose_reset_2026_10_13/aggregate_results.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-d9909e593723c658"></a>

### case_summary.csv

Percorso originale: `Trajectory Generator/validation/random_pose_reset_2026_10_13/case_summary.csv`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-dc025c313c648b99"></a>

### comparison_response.md

Percorso originale: `Trajectory Generator/validation/random_pose_reset_2026_10_13/claude/comparison_response.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-0acffef99124b01f"></a>

### source_response.md

Percorso originale: `Trajectory Generator/validation/random_pose_reset_2026_10_13/claude/source_response.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-142ba8cce3baa885"></a>

### timing_margin_adjudication.json

Percorso originale: `Trajectory Generator/validation/random_pose_reset_2026_10_13/claude/timing_margin_adjudication.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-18f5331ee5261ab7"></a>

### independent_saved_output_verification.md

Percorso originale: `Trajectory Generator/validation/random_pose_reset_2026_10_13/independent_saved_output_verification.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-40c691e84a135428"></a>

### source_protocol_audit.md

Percorso originale: `Trajectory Generator/validation/random_pose_reset_2026_10_13/source_protocol_audit.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-dbe35b50ba2605e5"></a>

### PROTOCOL.md

Percorso originale: `Trajectory Generator/validation/reset_ik_audit_2026_10_12/PROTOCOL.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-926d23f1034eb9fa"></a>

### comparison_response.md

Percorso originale: `Trajectory Generator/validation/reset_ik_audit_2026_10_12/claude/comparison_response.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-9b0407d87a4f6e41"></a>

### source_receipt.json

Percorso originale: `Trajectory Generator/validation/reset_ik_audit_2026_10_12/claude/source_receipt.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-32801c44eb48464a"></a>

### source_response.md

Percorso originale: `Trajectory Generator/validation/reset_ik_audit_2026_10_12/claude/source_response.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-bbf3dd9795cb439e"></a>

### codex_source_addendum.json

Percorso originale: `Trajectory Generator/validation/reset_ik_audit_2026_10_12/codex_source_addendum.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-23395efe65ff8df8"></a>

### codex_source_audit.json

Percorso originale: `Trajectory Generator/validation/reset_ik_audit_2026_10_12/codex_source_audit.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-d14b5a947e80d671"></a>

### comparison_synthesis.json

Percorso originale: `Trajectory Generator/validation/reset_ik_audit_2026_10_12/comparison_synthesis.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-0e0a324ef1132e0e"></a>

### consolidated_results.json

Percorso originale: `Trajectory Generator/validation/reset_ik_audit_2026_10_12/consolidated_results.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-94d9a2c7a259ee1a"></a>

### distribution_audit.md

Percorso originale: `Trajectory Generator/validation/reset_ik_audit_2026_10_12/distribution/distribution_audit.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-7f75654cb93080b6"></a>

### distribution_metrics.json

Percorso originale: `Trajectory Generator/validation/reset_ik_audit_2026_10_12/distribution/distribution_metrics.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-36d321a4924231ed"></a>

### independent_saved_outputs_review.md

Percorso originale: `Trajectory Generator/validation/reset_ik_audit_2026_10_12/independent_saved_outputs_review.md`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-c6091da15db50c0e"></a>

### privilege_invariance_receipt.json

Percorso originale: `Trajectory Generator/validation/reset_ik_audit_2026_10_12/privilege_invariance_receipt.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.

<a id="artifact-9ff584926faefe68"></a>

### reset_probe.json

Percorso originale: `Trajectory Generator/validation/reset_ik_audit_2026_10_12/rollouts/passive_reset_probe/reset_probe.json`.

Materiale rimosso nella pulizia del 2026-10-17; per recuperarlo cercare
il percorso originale nella copia esterna del progetto.
