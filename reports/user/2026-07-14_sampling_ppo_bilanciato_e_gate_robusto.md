# Sampling PPO interleaved, gate condition-matched e recovery da H0

## Esito sintetico

Il piano e stato eseguito fino alla sua condizione di arresto:

1. il sampling PPO e ora esattamente bilanciato e interleaved anche nel batch
   realmente visto dal Learner;
2. il gate usa reserve H0 separate per ogni condizione, senza confronti
   impropri tra start diversi;
3. un update PPO conservativo da H0 con learning rate `5e-7` ha imparato, ma e
   stato respinto per perdita di robustezza nel caso `+0,20 s / seed 123`;
4. una sola recovery adaptation pre-registrata, seguita da una sola proiezione
   verso H0, ha recuperato il caso stocastico critico ma ha peggiorato le
   reserve in due casi deterministici ed e quindi stata respinta.

Nessun checkpoint e stato promosso. Il checkpoint canonico resta:

`validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last`

Non sono state modificate reward, soglia di penetrazione GRF, criteri reserve,
feature deployable o semantica SEA.

## Chiarimento: "impara" non significa "diventa piu robusto"

Dire che l'update PPO **impara realmente** significa che:

- i pesi dell'actor cambiano;
- cambia la media delle azioni prodotte sulle stesse osservazioni;
- critic, loss e KL mostrano un aggiornamento numericamente effettivo;
- la log-standard-deviation resta congelata come richiesto.

Questo dimostra che il gradiente non e nullo e che il training non e un falso
restore. Non dimostra pero che il nuovo controller sia migliore closed-loop.
Una variazione molto piccola delle azioni puo spostare gli eventi di contatto,
portare il simulatore in stati diversi e restringere il bacino di recovery. E
esattamente cio che accade all'update PPO: tre rollout deterministici passano,
ma il caso stocastico sensibile termina a 212/500 step.

## Chiarimento: ripartire da H0 con sampling bilanciato

**Ripartire da H0** significa ripristinare il checkpoint RLlib completo, non
soltanto copiare i pesi actor. Vengono quindi recuperati actor, critic gia
riscaldato, stato dell'ottimizzatore e metadati compatibili; il nuovo update
parte dalla stessa baseline verificata.

**Sampling bilanciato** significa che ciascuno dei tre start contribuisce con
lo stesso numero di transizioni sia alla raccolta sia ai gradienti:

- start `1,7568709838 s`: 1.536 transizioni reali;
- start `1,9568709838 s`: 1.536 transizioni reali;
- start `2,1568709838 s`: 1.536 transizioni reali;
- totale: 4.608 transizioni.

Il solo conteggio degli EnvRunner non bastava. Dopo GAE Ray produceva 4.622
righe, includendo 14 righe bootstrap. La pipeline ora:

- usa `loss_mask` per conservare le sole transizioni reali;
- compatta 4.622 righe a 4.608;
- riordina in round-robin i tre start, con run-length massimo uguale a uno;
- riordina in modo transazionale tutte le colonne timestep-aligned, incluse
  strutture NumPy/Torch annidate;
- usa nove minibatch da 512 e una sola epoca, senza riuso di campioni.

Ogni minibatch contiene quindi `171/171/170` campioni dei tre start, con la
condizione da 170 ruotata tra i minibatch. La modalita esatta fallisce in modo
chiuso se `num_epochs != 1`, perche una seconda epoca Ray rimescolerebbe le
righe e invaliderebbe l'attestazione di interleaving.

## Gate reserve condition-matched

Il precedente cap globale derivato dal caso stocastico non era valido per
confrontare start differenti: H0 stesso raggiunge `624,679 Nm` nel
deterministico `+0,20 s`, piu del valore `596,197 Nm` del caso stocastico.

Il gate ora confronta ogni candidato con H0 nella **stessa** condizione:

| Condizione | Reserve H0 |
| --- | ---: |
| deterministico `-0,20 s` | 548,830304 Nm |
| deterministico nominale | 493,828205 Nm |
| deterministico `+0,20 s` | 624,679256 Nm |
| stocastico `+0,20 s / seed 123` | 596,196563 Nm |

Il contratto e `candidate <= H0 + max(1e-6 Nm, 1e-9 * H0)`. Le quattro
summary H0 sono validate rispetto ai contratti fisici e protette da SHA-256.
Questo non allenta il gate: nei primi due casi e piu severo del vecchio cap
globale e nel terzo evita di respingere perfino H0.

## Update PPO conservativo da H0

Run:

`Trajectory Generator/runs/training/validation/warm_start_h1_runs/2026-07-14_h0_exact_interleaved_lr5e-7_iter2_schema2/`

Audit del training:

| Voce | Valore |
| --- | ---: |
| step per start | `1536 / 1536 / 1536` |
| righe post-GAE | 4.622 |
| righe bootstrap rimosse | 14 |
| righe interleaved | 4.608 |
| run-length massimo dello start | 1 |
| minibatch | `9 x 512` |
| epoche | 1 |
| KL massimo sui minibatch | 0,001052326 |
| KL minimo | 0 |
| KL non finiti | 0 |

Il confronto H0 -> candidato conferma un update actor reale:

- RMS dei delta dei parametri actor: `6,282e-7`;
- massimo delta assoluto dei parametri: `3,718e-6`;
- action-mean RMSE: `9,890e-5`;
- massimo delta delle action mean: `2,900e-4`;
- KL empirico medio: `0,0003913`;
- KL empirico massimo: `0,0018577`;
- variazione log-standard-deviation: zero.

Gate:

`validation/robust_gate_runs/2026-07-14_h0_exact_interleaved_lr5e-7_iter2_schema2_condition_matched/robust_gate.json`

| Caso | Step | Cicli | Penetrazione | Reserve candidato / H0 | Esito |
| --- | ---: | ---: | ---: | ---: | --- |
| det. `-0,20 s` | 500 | 2 | 24,138929 mm | 547,604 / 548,830 Nm | PASS |
| det. nominale | 500 | 2 | 23,899916 mm | 493,776 / 493,828 Nm | PASS |
| det. `+0,20 s` | 500 | 2 | 24,281124 mm | 614,638 / 624,679 Nm | PASS |
| stoc. `+0,20 s`, seed 123 | 212 | 1 | 25,009760 mm | 673,420 / 596,197 Nm | **FAIL** |

Il learning rate dimezzato rende il fallimento meno grave del precedente
update `1e-6`, ma non lo elimina. Dimezzare ancora il learning rate non e una
strategia giustificata: il problema non e soltanto la grandezza globale
dell'update, ma la sua direzione rispetto al recovery basin.

## Unico esperimento di recovery da H0

### Dataset e limiti

La sorgente actor usata per l'adaptation e stata verificata bit-exact rispetto
a H0. Il dataset contiene:

- 8.000 anchor H0 da una trace pulita (`500 x 16` repliche);
- 105 stati realmente visitati prima del primo mismatch FSM nel rollout
  disturbato `+0,20 s / seed 123`;
- 105 righe interpolate tra quegli stati;
- 8.210 righe totali.

Le 210 righe recovery non sono 210 osservazioni temporalmente indipendenti e
lo split casuale puo condividere duplicati tra train e validation. Le label
sono inoltre le action mean di H0, non target prescritti o reward. Questo
rende l'esperimento un probe diagnostico, non una prova indipendente.

### Fit raw e proiezione unica

Il fit raw migliora l'RMSE offline recovery da `0,0433905` a `0,0402862`
(-7,15%), ma viola il guard di preservazione: shift massimo pulito
`0,007331 > 0,0005`. Il comando termina correttamente con esito fail-closed e
il candidato raw non viene sottoposto a rollout.

Artefatto raw:

`Trajectory Generator/runs/training/validation/target_domain_noise_adaptation/2026-07-14_h0_plus020_sigma0005_seed123_conservative/`

Come pre-registrato e stata applicata una sola proiezione deterministica verso
H0, senza nuovo training. Il coefficiente e `alpha=0,0733499862`; sulla trace
nominale usata come guard lo shift massimo e `0,000499994`. Il beneficio
recovery residuo e pero soltanto circa `0,53%`.

Artefatto proiettato:

`Trajectory Generator/runs/training/validation/target_domain_noise_adaptation/2026-07-14_h0_plus020_sigma0005_seed123_projected_nominal5e4/rl_module_projected`

Sulla trace pulita `+0,20 s`, non usata per calibrare il guard nominale, il
massimo shift arriva a `5,366e-4` e il KL massimo a `0,010882`. La logstd e
tutti i pesi non-actor restano invariati.

### Gate dinamico della proiezione

Tutti e quattro i rollout completano 500 step, almeno due cicli, penetrazione
inferiore a 25 mm e zero clipping. Il gate condition-matched fallisce tuttavia
sulle reserve:

| Caso | Step | Cicli | Penetrazione | Reserve candidato | Delta vs H0 | Esito |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| det. `-0,20 s` | 500 | 2 | 24,402724 mm | 549,056596 Nm | +0,226292 Nm | **FAIL** |
| det. nominale | 500 | 3 | 24,129376 mm | 493,295132 Nm | -0,533073 Nm | PASS |
| det. `+0,20 s` | 500 | 2 | 24,133223 mm | 627,676059 Nm | +2,996802 Nm | **FAIL** |
| stoc. `+0,20 s`, seed 123 | 500 | 3 | 24,548152 mm | 587,300894 Nm | -8,895669 Nm | PASS |

Il PASS stocastico mostra che la direzione recovery ha un effetto closed-loop
reale: il candidato supera il punto in cui PPO falliva. Tuttavia seed 123 e
stato usato per costruire il dataset e non e held-out. Inoltre due condizioni
deterministiche regrediscono sulle reserve. Il gate complessivo e quindi
`FAIL` e, come stabilito prima dei rollout, i seed held-out non vengono eseguiti
dopo un fallimento della matrice base.

Artefatto del gate recovery:

`Trajectory Generator/runs/rollout/validation/recovery_gate_runs/2026-07-14_h0_self_recovery_projected_nominal5e4/recovery_gate.json`

## File modificati

- `Trajectory Generator/baseline_MLP/start_sampling.py`
- `Trajectory Generator/baseline_MLP/start_condition_metrics.py`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/tb_logging.py`
- `Trajectory Generator/baseline_MLP/training_config.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `validation/robust_ppo_gate.py`
- `validation/test_start_sampling.py`
- `validation/test_start_condition_metrics.py`
- `validation/test_tb_logging_start_metrics.py`
- `validation/test_training_start_balance.py`
- `validation/test_robust_ppo_gate.py`
- `validation/test_rollout_eval.py`

Nessuna modifica al plugin C++, al modello SEA, alla reward o alle soglie
fisiche. Gli script di target-domain adaptation e proiezione sono stati usati,
non modificati.

## Test e verifiche

- suite completa `validation/test_*.py`: 131/131 passati;
- smoke test configurazione: 75/75 controlli passati;
- test focalizzati target-domain: 10/10 passati;
- `py_compile` sui file Python coinvolti: passato;
- `git diff --check`: passato;
- restore completo da H0: verificato;
- attore sorgente adaptation vs H0: bit-exact;
- save/reload della proiezione: bit-exact;
- logstd e non-actor della proiezione: invariati;
- nessuna promozione e nessuna copia del candidato.

## Decisione e TODO

Entrambi i candidati sono **REJECT**. H0 resta checkpoint canonico e rollback.

TODO vincolanti per un futuro esperimento:

- mantenere sampling interleaved, singola epoca e gate condition-matched;
- non proseguire con ulteriori dimezzamenti della LR o altre proiezioni sulla
  stessa trace/seed: l'evidenza disponibile non li giustifica;
- raccogliere dati recovery realmente event-aligned e indipendenti, includendo
  la regione successiva al mismatch FSM invece di sole interpolazioni pre-evento;
- separare rigorosamente seed di costruzione, validazione e gate held-out;
- eseguire i seed held-out soltanto dopo un PASS completo della matrice base;
- non allentare reward, 25 mm, reserve condition-matched o contratto deployable;
- promuovere un checkpoint solo dopo PASS di tutti i gate pre-registrati.
