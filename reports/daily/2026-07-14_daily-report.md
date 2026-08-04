# Daily Report - 2026-07-14

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `reports/user/2026-07-14_capacita_learning_e_robustezza_multistart.md`
- `reports/user/2026-07-14_sampling_ppo_bilanciato_e_gate_robusto.md`

## Sintesi

La giornata ha verificato se PPO fosse realmente capace di allontanarsi dal
warm start imitativo senza perdere robustezza. Lo sweep e la breve catena PPO
hanno dimostrato che anche LR `1e-6` modifica realmente la policy e genera
traiettorie closed-loop sostanzialmente diverse dal prescribed.

Il limite non e' quindi mancanza di learning. Il problema e' che i gate
deterministici e le metriche medie non rilevano la riduzione del recovery basin
su uno start specifico. Nessun checkpoint actor aggiornato e' stato promosso;
il checkpoint H0 con critic warm-up resta la baseline canonica.

## 1. Contratto e strumenti

Sono rimasti invariati:

- actor Markov a 35 feature deployable;
- critic a 84 feature con osservazioni privilegiate;
- reward `ex_novo`, senza termine imitativo;
- `sigma=0.005` e `freeze_logstd=true` durante PPO;
- FSM, GRF online e hard guard a 25 mm;
- prescribed usato solo come prior warm-start e diagnostica.

Sono stati aggiunti strumenti riproducibili per:

- confrontare pesi, azioni medie e KL empirico tra checkpoint;
- confrontare trace closed-loop, cinematica, reference, reward e penetrazione;
- ricostruire il rumore effettivo nei rollout stocastici;
- verificare manifest e larghezza delle feature actor nei checkpoint esportati.

## 2. Sweep del learning rate da H0

Ogni candidato e' partito dallo stesso checkpoint H0.

| LR | KL empirico | H2 deterministico | Decisione |
| ---: | ---: | --- | --- |
| `1e-6` | `0.000548` | 3/3 start, aggregate `151.862` | candidato |
| `2e-6` | `0.002455` | `+0.20 s` fallito a 215 step | respinto |
| `5e-6` | non eseguito | salto precedente gia' instabile | stop |

La fascia KL generica `0.002-0.01` e' risultata troppo permissiva per il
sistema: `2e-6` era formalmente dentro quella fascia ma ha superato il guard a
`25.144 mm`.

## 3. Catena PPO breve a LR 1e-6

La catena e' stata estesa fino alla logical iteration 5, mantenendo ogni
checkpoint come rollback.

| Checkpoint | KL cumulativo da H0 | Gate | Esito |
| --- | ---: | --- | --- |
| iter2 | `0.000548` | H2 tre start | 3/3, aggregate `151.862` |
| iter3 | `0.001862` | `-0.20 s` | 500 step |
| iter4 | `0.004328` | `+0.20 s` | 500 step |
| iter5 | `0.007856` | H2 tre start | 3/3, aggregate `147.331` |

Iter5 e' rimasta valida deterministicamente, ma la reward on-policy e' scesa a
`-4.779` e l'aggregate H2 e' risultato peggiore di iter2 e H1-bis. La catena
e' stata arrestata senza ridurre ulteriormente LR o sigma.

## 4. Distanza dal warm start e dal prescribed

La policy non e' rimasta congelata. Su `-0.20 s`, iter5 rispetto a H1-bis ha
prodotto:

```text
differenza RMS azioni                 0.156 rad
differenza RMS cinematica             0.110 rad
differenza RMS reference servita      0.109 rad
```

La cinematica di iter5 resta inoltre distante dal prescribed di
`0.353-0.378 rad` RMS sui tre start. Piccoli delta dei pesi sono amplificati
dalla dinamica closed-loop e producono traiettorie ex-novo reali. Abbassare la
LR non ha quindi appiattito la rete sul pre-training.

## 5. Robustezza a sigma 0.0075

Una copia diagnostica di iter2, con sola bias `log-std` modificata e media
bit-identica, ha completato il nominale su tre seed:

```text
successo nominale                 3/3
reward media                      56.819
RMS rumore reale                  0.007327
clipping                          0
```

Sul seed 123 multi-start:

```text
-0.20 s                   500 step, 2 cicli
+0.20 s                   210 step, penetrazione 25.166 mm
```

Il rumore held per 50 ms ha peggiorato lo start `+0.20 s`, fermandosi a 39
step. La fragilita' non dipende soltanto dal dithering ad alta frequenza.

## 6. Test causale H0 contro iter2

Con stesso start `+0.20 s`, seed 123 e `sigma=0.005`:

| Actor | Step | Cicli | Return | Penetrazione |
| --- | ---: | ---: | ---: | ---: |
| H0 | 500 | 3 | 57.108 | 24.559 mm |
| iter2 | 214 | 0 | -35.194 | 25.172 mm |

H0 fallisce sullo stesso caso solo quando portato a `sigma=0.0075`. Esistono
quindi due limiti distinti:

1. H0 ha un recovery basin compreso tra sigma `0.005` e `0.0075` sullo start
   positivo;
2. il primo update PPO restringe ulteriormente quel basin e rende fragile
   anche `sigma=0.005`.

Il gate deterministico aveva accettato iter2 perche' vedeva un miglioramento
medio, ma non includeva il worst case stocastico.

## 7. Sampling PPO esattamente bilanciato nel Learner

La seconda parte della giornata ha corretto il contratto di sampling, andando
oltre il semplice bilanciamento del numero di EnvRunner. Ogni update usa ora
esattamente:

| Start | Transizioni reali |
| --- | ---: |
| `1,7568709838 s` | 1.536 |
| `1,9568709838 s` | 1.536 |
| `2,1568709838 s` | 1.536 |
| **Totale** | **4.608** |

Ray produceva inizialmente 4.622 righe post-GAE, comprendenti 14 righe di
bootstrap. La pipeline introdotta:

- conserva soltanto le transizioni reali tramite `loss_mask`;
- compatta il batch da 4.622 a 4.608 righe;
- riordina tutte le colonne timestep-aligned in round-robin fra i tre start;
- limita a uno il massimo run-length della stessa condizione;
- usa nove minibatch da 512 e una sola epoca.

Ogni minibatch contiene `171/171/170` campioni dei tre start, ruotando la
condizione da 170. La modalita' exact-start fallisce in modo chiuso se i
conteggi, la compaction, l'interleaving o il numero di epoche non rispettano il
contratto.

## 8. Gate reserve condition-matched e update PPO conservativo

Il precedente cap globale delle reserve e' stato sostituito con confronti fra
candidato e H0 nella stessa condizione:

| Condizione | Reserve H0 |
| --- | ---: |
| deterministico `-0,20 s` | 548,830304 Nm |
| deterministico nominale | 493,828205 Nm |
| deterministico `+0,20 s` | 624,679256 Nm |
| stocastico `+0,20 s / seed 123` | 596,196563 Nm |

L'update PPO da H0 con learning rate `5e-7` e' risultato numericamente reale e
controllato:

- RMS del delta dei parametri actor: `6,282e-7`;
- massimo delta assoluto: `3,718e-6`;
- action-mean RMSE: `9,890e-5`;
- KL empirico medio: `0,0003913`;
- KL massimo sui minibatch: `0,001052326`;
- log-standard-deviation invariata.

I tre rollout deterministici completano 500 step e rispettano i cap reserve.
Il caso stocastico `+0,20 s / seed 123` termina invece a 212 step, con
penetrazione `25,009760 mm` e reserve `673,419582 Nm`. Il candidato viene
quindi respinto: l'actor ha imparato, ma ha ristretto il proprio bacino di
recovery.

## 9. Unico esperimento di recovery preregistrato

La recovery adaptation e' ripartita da un actor bit-exact a H0. Il dataset
contiene 8.000 ancore H0 e 210 righe recovery derivate da 105 stati visitati
prima del mismatch FSM. Il fit raw migliora l'RMSE offline del `7,15%`, ma
viola il guard di preservazione e viene fermato prima dei rollout.

Come preregistrato, e' stata applicata una sola proiezione deterministica verso
H0, con `alpha=0,0733499862`. Il candidato proiettato recupera il caso critico
stocastico e completa tutti e quattro i rollout, ma fallisce il gate reserve:

| Condizione | Delta reserve rispetto a H0 | Esito |
| --- | ---: | --- |
| deterministico `-0,20 s` | `+0,226292 Nm` | FAIL |
| deterministico nominale | `-0,533073 Nm` | PASS |
| deterministico `+0,20 s` | `+2,996802 Nm` | FAIL |
| stocastico `+0,20 s / seed 123` | `-8,895669 Nm` | PASS |

Il miglioramento del caso critico e' reale, ma seed e trace erano parte della
costruzione del dataset e due condizioni deterministiche regrediscono. Anche
questo candidato e' stato respinto e nessun seed held-out e' stato aperto.

## Decisione finale

Nessun checkpoint actor aggiornato e' stato promosso. La baseline canonica
resta:

```text
validation/critic_warmup/
2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last
```

Questa decisione non riporta la policy al prescribed: conserva il warm start
come prior, il critic privilegiato e la reward ex-novo, ma rifiuta update PPO
che imparano a scapito della robustezza stocastica.

## File modificati

- `Trajectory Generator/baseline_MLP/configure_actor_exploration.py`
- `validation/compare_policy_checkpoints.py`
- `validation/compare_rollout_traces.py`
- `validation/analyze_stochastic_rollouts.py`
- `validation/test_actor_exploration_configuration.py`
- `validation/test_compare_policy_checkpoints.py`
- `validation/test_compare_rollout_traces.py`
- `validation/test_analyze_stochastic_rollouts.py`

Ulteriori file modificati nella seconda parte della giornata:

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

Non sono stati modificati plugin C++, modello SEA, reward o soglie fisiche.

## Test e verifiche

- test focalizzati configuratori e analizzatori: `18/18` PASS;
- audit esatto della media durante la modifica di sigma: PASS;
- audit optimizer Adam e LR dopo ogni restore: PASS;
- H2 deterministico sui tre start: completato;
- probe stocastici multi-seed e multi-start: completati;
- confronto H0 -> iter2 a stesso seed, start e sigma: completato;
- `py_compile`: PASS;
- `git diff --check`: PASS;
- nessun processo training o rollout lasciato attivo.

Verifiche aggiuntive del sampling e della recovery:

- suite completa `validation/test_*.py`: `131/131` PASS;
- smoke configurazione: `75/75` PASS;
- test focalizzati target-domain: `10/10` PASS;
- restore completo da H0: PASS;
- actor sorgente adaptation bit-exact a H0: PASS;
- save/reload della proiezione bit-exact: PASS;
- logstd e pesi non-actor della proiezione invariati: PASS;
- nessuna promozione o copia del candidato: verificato.

## TODO chiusi il 14/7

- [x] Ripartire dal full checkpoint H0, non da iter2-iter5.
- [x] Inserire nel gate i tre start deterministici e il worst case
      `+0,20 s / seed 123` a `sigma=0.005`.
- [x] Bilanciare esattamente i tre start anche nel batch realmente visto dal
      Learner.
- [x] Rimuovere le righe bootstrap e attestare l'interleaving round-robin.
- [x] Rendere il gate reserve condition-matched rispetto a H0.
- [x] Monitorare il picco reserve insieme a durata, cicli, penetrazione e
      clipping.
- [x] Verificare una recovery preregistrata senza promuovere il candidato in
      caso di regressione.

## TODO ancora aperti e propagati

- [ ] Analizzare return e advantage separatamente per start, evitando che la
      media globale sacrifichi il worst case.
- [ ] Solo dopo un gate completo a `sigma=0.005`, riprovare `sigma=0.0075` su
      almeno tre seed per ciascuno dei tre start.
- [ ] Non eseguire altri dimezzamenti della learning rate o altre proiezioni
      sulla stessa trace/seed: l'evidenza corrente non li giustifica.
- [ ] Raccogliere recovery data realmente event-aligned e indipendenti,
      includendo la regione successiva al mismatch FSM.
- [ ] Separare rigorosamente seed di costruzione, validation e gate held-out.
- [ ] Eseguire gli held-out soltanto dopo un PASS completo della matrice base.
- [ ] Mantenere sampling interleaved, singola epoca e reserve
      condition-matched nei prossimi esperimenti.
- [ ] Non allentare reward, soglia da 25 mm, cap reserve o contratto actor per
      forzare un PASS.
- [ ] Promuovere un checkpoint soltanto dopo il PASS di tutti i gate
      preregistrati.
- [ ] Estendere la validazione a trial, velocita' e soggetti differenti prima
      di dichiarare deployment validato.
- [ ] La memoria Markov del controller e' presente e validata; una memoria
      ricorrente resta differita finche' non emerge un limite sequenziale.
- [ ] Spiegare il TO precoce rifiutato nella seconda stance dell'oracolo
      multi-ciclo, TODO storico ancora aperto.
- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta,
      TODO storico ancora aperto dal 13/6.
