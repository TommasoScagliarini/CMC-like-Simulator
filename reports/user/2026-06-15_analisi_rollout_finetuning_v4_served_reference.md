# Analisi rollout del fine-tuning V4 della served reference

**Data:** 2026-06-15

## Problema

Il training imitativo V4 e stato prolungato a episodi da 5 secondi, aumentando
il peso relativo della qualita della reference servita rispetto al movimento
fisico effettivo:

```yaml
reward:
  imitation_weight: 4.0
  served_imitation_weight: 8.0
  blend_served_imitation: 0.80
  blend_imitation: 0.20
  blend_imitation_tracking: 0
```

L'obiettivo dell'analisi era verificare se il fine-tuning avesse migliorato la
forma della served reference su piu cicli del passo, senza perdere stabilita o
qualita di inseguimento dei SEA.

## Strategia

1. Verificato il completamento del fine-tuning e selezionato il checkpoint
   migliore, corrispondente all'iterazione logica `52`.
2. Eseguito un rollout deterministico da `rl_module_best`, con registrazione
   completa degli output `.sto`.
3. Confrontato il nuovo rollout con il rollout V4 precedente sul medesimo
   intervallo iniziale di 2 secondi.
4. Analizzati separatamente:
   - qualita served-reference rispetto al target imitativo;
   - tracking fisico della served reference;
   - errore di coppia SEA;
   - alternanza dei comandi;
   - stabilita, contatto e penetrazione.
5. Verificata la semantica delle azioni durante training e rollout.

## Training

Il fine-tuning e terminato correttamente all'iterazione logica `57`.

- Best checkpoint: iterazione `52`.
- Best episode return medio di training: `236.27`.
- Episode length medio al best: `497.5` step.
- L'iterazione `56` ha raggiunto una lunghezza media di `501` step.
- L'ultimo checkpoint, iterazione `57`, e peggiore del best:
  return medio `202.75`, episode length medio `467.55` e una divergenza del
  ginocchio protesico.

Per l'inference e stato quindi usato:

```text
Trajectory Generator/runs/baseline_mlp_imit_v4_c2_4hz_finetune/rl_module_best
```

## Risultato del rollout

Il rollout deterministico ha completato l'intero episodio:

- durata: `5.0 s`;
- step: `501`;
- episode return: `243.71`;
- reward media: `0.486`;
- terminazione: normale `episode_time_limit`;
- nessuna caduta o divergenza;
- altezza minima pelvis: circa `0.952 m`;
- nessuna saturazione dell'ingresso motore SEA.

Output analizzati:

```text
Trajectory Generator/runs/baseline_mlp_imit_v4_c2_4hz_finetune_rollout_clipped
```

## Qualita della served reference

### Intero episodio da 5 secondi

| Metrica | Ginocchio | Caviglia |
|---|---:|---:|
| RMSE served-target imitativo | `0.229 rad` (`13.1 deg`) | `0.182 rad` (`10.5 deg`) |
| Correlazione served-target | `0.644` | `0.436` |
| RMSE movimento reale-served | `0.0029 rad` (`0.17 deg`) | `0.0069 rad` (`0.40 deg`) |
| RMSE errore coppia SEA | `1.69 Nm` | `0.24 Nm` |

I controllori SEA inseguono quindi con precisione la reference servita. La
principale sorgente di errore resta la traiettoria scelta dalla policy e
successivamente prodotta dal reference governor, non il tracking fisico.

La qualita della served reference e buona soprattutto nel primo secondo. Negli
intervalli successivi l'errore cresce e la caviglia sviluppa una marcata
polarizzazione positiva rispetto al target imitativo:

| Intervallo | Knee RMSE | Ankle RMSE | Ankle bias |
|---|---:|---:|---:|
| `0-1 s` | `0.071 rad` | `0.043 rad` | `-0.015 rad` |
| `1-2 s` | `0.298 rad` | `0.188 rad` | `+0.141 rad` |
| `2-3 s` | `0.220 rad` | `0.157 rad` | `+0.140 rad` |
| `3-4 s` | `0.206 rad` | `0.235 rad` | `+0.204 rad` |
| `4-5 s` | `0.277 rad` | `0.222 rad` | `+0.180 rad` |

## Confronto con il rollout V4 precedente

Il confronto seguente usa soltanto i primi 2 secondi del nuovo rollout, per
evitare che la diversa durata alteri le conclusioni.

| Metrica | V4 precedente | Fine-tuning | Variazione |
|---|---:|---:|---:|
| Knee served-target RMSE | `0.221 rad` | `0.217 rad` | `-1.9%` |
| Ankle served-target RMSE | `0.136 rad` | `0.137 rad` | `+0.9%` |
| Knee actual-served RMSE | `0.00394 rad` | `0.00367 rad` | `-6.8%` |
| Ankle actual-served RMSE | `0.00863 rad` | `0.00788 rad` | `-8.7%` |
| Knee torque-error RMSE | `0.95 Nm` | `1.30 Nm` | `+36.3%` |
| Ankle torque-error RMSE | `0.27 Nm` | `0.29 Nm` | `+7.1%` |

Il fine-tuning ha mantenuto la stabilita e migliorato leggermente il tracking
fisico, ma non ha prodotto un miglioramento significativo della forma della
served reference.

## Alternanza dei comandi

Nei primi 2 secondi il fine-tuning riduce l'ampiezza media delle variazioni del
comando caviglia di circa il `41%`. Sull'intero episodio resta tuttavia una
forte alternanza prossima a `50 Hz`:

- inversioni consecutive del comando caviglia: circa `93%`;
- energia del comando caviglia sopra `20 Hz`: circa `81%`;
- variazione media inter-step dell'endpoint caviglia: `0.450 rad`;
- variazione media inter-step dell'endpoint ginocchio: `0.249 rad`.

Il reference governor filtra fortemente questa alternanza, motivo per cui il
movimento fisico continua a seguire una reference servita regolare. Il
chattering raw resta quindi una inefficienza della policy e una possibile
causa indiretta di scarsa controllabilita della forma, ma non e il problema
fisico dominante osservato nel rollout.

## Clipping delle azioni

Durante l'analisi e stato osservato che l'azione deterministica grezza della
rete raggiunge `|a|max = 1.883`, oltre lo spazio dichiarato `[-1, 1]`.

La verifica del codice ha mostrato che `FlattenClipAction` applicava gia
correttamente il clipping prima di inviare l'azione al simulatore. Il rollout
iniziale era quindi fisicamente valido. Tuttavia, il trace registrava soltanto
l'azione grezza e non rendeva evidente l'azione effettivamente applicata.

`rollout_eval.py` e stato aggiornato per:

- registrare separatamente `raw_policy_action` e `applied_policy_action`;
- riportare `applied_action_abs_max`;
- contare gli step e la frazione di valori soggetti a clipping.

Nel rollout analizzato:

- azione applicata massima: `1.0`;
- step con almeno un valore clippato: `119 / 501`;
- valori clippati: circa `11.9%` del totale;
- il clipping riguarda principalmente la caviglia.

I file fisici prodotti prima e dopo questa modifica sono risultati identici
byte per byte, confermando che il wrapper dell'ambiente applicava gia la
semantica corretta.

## Contatto e reclutamento

- Penetrazione massima onlineGRF sinistra: circa `24 mm`.
- Penetrazione sinistra oltre `20 mm`: circa `5.2%` del rollout.
- Penetrazione massima destra: circa `36 mm`.
- Penetrazione destra oltre `20 mm`: circa `33.0%`.
- Muscle share medio: circa `0.236`.
- Norma media delle coppie reserve: circa `365 Nm`.

Questi indicatori non sono stati identificati come causa primaria della scarsa
forma della served reference, ma mostrano che la qualita globale della dinamica
e del contatto resta migliorabile.

## Soluzione e decisione

Il checkpoint dell'iterazione `52` viene mantenuto come baseline stabile del
fine-tuning V4. Non e consigliato continuare semplicemente lo stesso training:
il vantaggio rispetto al rollout precedente e marginale sulla metrica
principale served-target e degrada sugli intervalli piu lunghi.

La prossima modifica dovrebbe intervenire sull'obiettivo o sulla struttura che
determina la served reference, mantenendo separato il problema del tracking SEA,
che risulta gia risolto con buona precisione.

## File modificati

- `Trajectory Generator/baseline_MLP/rollout_eval.py`
  - trace separato delle azioni raw e applicate;
  - metriche esplicite sul clipping.

## Verifiche eseguite

- Rollout deterministico completo da 5 secondi dal best checkpoint.
- Secondo rollout dopo l'aggiornamento diagnostico.
- Confronto byte per byte degli output cinematici, reference e diagnostica SEA:
  identici tra i due rollout.
- Confronto numerico con il rollout V4 precedente sui primi 2 secondi.
- Analisi spettrale e delle inversioni consecutive dei comandi.
- Verifica `python -m py_compile baseline_MLP/rollout_eval.py`: superata.
- Verifica `git diff --check`: superata; presente solo avviso Git sul futuro
  adattamento LF/CRLF.

## Comandi utili

Plot diagnostici:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-like python "plot\plotter.py" --results-dir "Trajectory Generator\runs\baseline_mlp_imit_v4_c2_4hz_finetune_rollout_clipped\sim_outputs" --prefix "rollout_episode" --setup "models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500_pi_setup.xml" --gait-side left
```

Visualizzazione:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-like python visualize.py --sto "Trajectory Generator\runs\baseline_mlp_imit_v4_c2_4hz_finetune_rollout_clipped\sim_outputs\rollout_episode_kinematics.sto" --speed 1.0
```

## TODO

- Decidere la prossima modifica specifica per migliorare la forma globale della
  served reference, evitando una semplice prosecuzione del training invariato.
- Valutare una penalita inter-step esplicita sul comando raw o sull'endpoint
  consecutivo dopo aver definito il nuovo obiettivo sulla forma.
- Indagare la polarizzazione positiva della served reference della caviglia
  dopo il primo secondo.
- Valutare separatamente il problema delle penetrazioni e dell'elevato uso
  delle reserve, senza confonderlo con il tracking SEA.
