# Audit offline finale del rumore — 2026-09-22

Analisi delle tracce salvate: nessun rollout aggiuntivo, nessuna modifica al training.

| Checkpoint | Deterministico return | Completamenti sigma 1 | Completamenti sigma 0.5 | Return medio sigma 1 | Return medio sigma 0.5 |
|---|---:|---:|---:|---:|---:|
| 40 | 222.15 | 5/5 | 5/5 | 231.84 | 284.20 |
| 60 | 254.16 | 4/5 | 4/5 | 264.50 | 271.04 |
| 80 | 203.83 | 2/5 | 5/5 | 265.17 | 290.64 |
| 100 | 229.02 | 3/5 | 5/5 | 191.37 | 293.80 |

Alla 100 i fallimenti sigma 1 sono seed 126 (297 passi) e 127 (176 passi), entrambi per penetrazione; i corrispondenti sigma 0.5 completano 500 passi. Nel test 80 i tre fallimenti sigma 1 erano tardivi (467–490 passi).

Sul prefisso comune di 176 passi, tutte e cinque le coppie alla 100 hanno RMSE minore con sigma 0.5, sia al ginocchio sia alla caviglia. Questo evita il confronto improprio fra RMSE di episodi brevi e completi, ma riguarda soltanto i primi 1,76 s.

## Distribuzione e clipping alla 100

| Scala rumore | Componenti tagliate | RMSE ginocchio 176 passi | RMSE caviglia 176 passi |
|---|---:|---:|---:|
| 0 | 15.70% | 5.846° | 2.877° |
| 0.5 | 26.96% | 5.484° | 2.733° |
| 1 | 37.20% | 7.679° | 4.128° |

Alla 100, sulle sole traiettorie sigma 1 (statistiche pesate per passi):

- pros_knee_angle: std appresa mediana 0.615, 95° percentile 3.724; media gaussiana fuori [-1, 1] nel 24.2% dei passi. Su questi stessi stati, dimezzare sigma cambia la media dell'azione dopo clipping di 0.078 unità normalizzate in valore assoluto medio (equivalente a 3.34° dell'endpoint grezzo; il filtro successivo ne modifica l'effetto fisico).
- pros_ankle_angle: std appresa mediana 0.648, 95° percentile 3.520; media gaussiana fuori [-1, 1] nel 23.9% dei passi. Su questi stessi stati, dimezzare sigma cambia la media dell'azione dopo clipping di 0.083 unità normalizzate in valore assoluto medio (equivalente a 3.33° dell'endpoint grezzo; il filtro successivo ne modifica l'effetto fisico).

## Interpretazione e test discriminante

Il dimezzamento del rumore è un fattore promettente: alla 100 migliora il tracking sul prefisso condiviso e salva i due fallimenti osservati. Non identifica tuttavia la sola varianza come causa, perché il clipping sposta anche la media applicata; inoltre 5 semi, unico stato iniziale e una sola replica di training non consentono conclusioni di robustezza generale.

Prima di modificare il training: rollout deterministici che applichino E[clip(mu+sigma*z)] ed E[clip(mu+0.5*sigma*z)] alle osservazioni via via raggiunte, confrontati con clip(mu). I tre casi separano la distorsione della media prodotta dal clipping dalla variabilità casuale. Poi valutazione accoppiata su più seed per confermare il vantaggio della scala 0.5; scegliere anche checkpoint 60 e best 72, perché last 100 non è automaticamente il migliore.

La quantità analitica sopra è calcolata sulle stesse osservazioni salvate e prova la modifica del comando medio, non l’esito di simulazioni controfattuali. Non attribuire tutto il problema al rumore: anche la policy deterministica 100 mantiene errore caviglia 4,915°, un solo ciclo FSM valido, e non uguaglia V26.

## Limiti

- Five repeated noise seeds and one fixed environment start; not independent training replicas or an equivalence test.
- Learned std is state dependent. Statistics on different executed trajectories include state-visitation differences.
- Noise scaling changes the expectation of the clipped action as well as its variance.
- Common-prefix RMSE ends at 176 steps for the final comparison; it excludes late phases and does not summarize whole-episode gait quality.
- Aggregate full-episode RMSE for failing and complete trajectories has survival/time-window confounding.

## Stessa osservazione iniziale: confronto pulito fra checkpoint

Le 39 componenti dell’osservazione iniziale risultano esattamente uguali nei quattro checkpoint. In questo singolo stato la deviazione standard aumenta, soprattutto alla caviglia, e la media della caviglia supera il limite +1 già a 60. È una modifica della distribuzione appresa, senza il confondente delle diverse traiettorie successive.

| Iterazione | Media ginocchio | Media caviglia | Std ginocchio | Std caviglia |
|---|---:|---:|---:|---:|
| 40 | 0.335 | 0.704 | 0.482 | 0.989 |
| 60 | 0.337 | 1.051 | 0.483 | 1.177 |
| 80 | 0.455 | 1.263 | 0.591 | 1.399 |
| 100 | 0.508 | 1.480 | 0.787 | 2.136 |

Questo singolo stato non rappresenta da solo la distribuzione dei passi né dimostra la causa dei fallimenti.

## Controllo storico V26

L’audit salvato il 20 settembre, già eseguito sulle osservazioni deterministiche del V26 best 87, riporta:

- Std appresa media ginocchio / caviglia: 1,026 / 0,750.
- Clipping deterministico per componente: 10,2% / 0,4%.
- Probabilità gaussiana condizionata di superare i limiti: 36,3% / 22,7%.
- RMS fra endpoint atteso dopo clipping e endpoint deterministico: 7,04° / 5,42°.

Quindi anche il V26 riuscito aveva una distribuzione ampia e una differenza fra comando deterministico e comando medio esplorativo. Questo meccanismo non basta da solo a spiegare il divario attuale. Le statistiche storiche sono calcolate sugli stati del V26, non sulle stesse osservazioni della unified39; non sono frequenze misurate in nuovi rollout stocastici.

Fonte: `validation/ablation_report_2026-09-20/metrics.json`, `historical.V26_aug20.policy_audit`. Nessuna nuova forward o simulazione è stata eseguita.
