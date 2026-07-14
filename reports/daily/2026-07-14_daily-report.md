# Daily Report - 2026-07-14

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidato

- `reports/user/2026-07-14_capacita_learning_e_robustezza_multistart.md`

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

## TODO aperti e propagati

- [ ] Ripartire da H0, non da iter2-iter5.
- [ ] Inserire nel gate di ogni update i tre start deterministici e almeno il
      worst case `+0.20 s`, seed 123, stocastico a `sigma=0.005`.
- [ ] Bilanciare esattamente il sampling dei tre start; usare un numero di
      EnvRunner multiplo di tre o una riduzione esplicita per start.
- [ ] Analizzare return e advantage separatamente per start, evitando che la
      media globale sacrifichi il worst case.
- [ ] Solo dopo un gate completo a `sigma=0.005`, riprovare `sigma=0.0075` su
      almeno tre seed per ciascuno dei tre start.
- [ ] Se nessun update supera il gate, intervenire sul protocollo di
      ottimizzazione robusta o su dati on-policy di recovery. Non modificare
      reward, soglia da 25 mm o informazioni actor per forzare il PASS.
- [ ] Monitorare e limitare esplicitamente il picco reserve nei futuri H2,
      oltre a durata, cicli, penetrazione e clipping.
- [ ] Estendere la validazione a trial, velocita' e soggetti differenti prima
      di dichiarare deployment validato.
- [ ] La memoria Markov del controller e' presente e validata; una memoria
      ricorrente resta differita finche' non emerge un limite sequenziale.
- [ ] Spiegare il TO precoce rifiutato nella seconda stance dell'oracolo
      multi-ciclo, TODO storico ancora aperto.
