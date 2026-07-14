# Capacita di learning e robustezza multi-start del warm start

## Problema

I cicli precedenti avevano ottenuto locomozione valida riducendo prima `sigma` e
poi il learning rate. Il rischio da verificare era che PPO fosse diventato una
quasi-replica del pre-training imitativo, incapace di cercare traiettorie ex
novo, oppure che piccoli aggiornamenti nascondessero una fragilita closed-loop.

Il criterio sperimentale adottato e stato quindi piu forte del semplice
`500/500 step`:

- PPO deve cambiare realmente la policy warm-start;
- la reward deve rimanere `ex_novo`;
- la locomozione deve restare valida sui tre start `-0,20`, nominale e `+0,20`;
- il checkpoint deve tollerare il rumore stocastico usato dal training;
- la distanza dal prescribed deve essere misurata solo come diagnostica, non
  usata come obiettivo o scorciatoia.

## Contratto mantenuto

- actor: 35 feature deployable e Markoviane;
- critic: 84 feature, incluse le osservazioni privilegiate previste;
- reward: `reward_mode=ex_novo`;
- `sigma=0,005` e `freeze_logstd=true` durante gli update PPO;
- FSM e GRF online;
- soglia di terminazione per penetrazione: 25 mm;
- nessun termine imitativo attivo nella reward ex novo.

## Strumenti diagnostici aggiunti

Sono stati aggiunti due confronti riproducibili:

- `validation/compare_policy_checkpoints.py`: delta dei pesi actor, differenza
  delle azioni medie e KL empirico su 1.500 osservazioni fisse;
- `validation/compare_rollout_traces.py`: differenza closed-loop di azioni,
  cinematica protesica, reference servita, reward, cicli e penetrazione.

E stato inoltre aggiunto `validation/analyze_stochastic_rollouts.py`, che
ricostruisce offline la media dell'actor sulle osservazioni registrate e misura
il rumore effettivo come `azione campionata - media`. Questo risolve la lacuna
del riepilogo standard di `rollout_eval`, che lascia nullo l'RMS per la modalita
stocastica RLlib non-held.

`configure_actor_exploration.py` ora accetta anche un manifest delle feature
esplicito. Il manifest viene verificato contro la larghezza reale dell'input
actor e ne viene registrato l'hash. La modifica era necessaria perche i
checkpoint PPO esportati conservano config e pesi, ma non sempre copiano il
manifest adiacente.

## Sweep del learning rate dallo stesso H0

### `LR=1e-6`

- KL RLlib: `0,000787`;
- KL empirico fisso H0 -> candidato: `0,000548`;
- shift RMS delle azioni medie: `0,000117`;
- sigma rimasto esattamente `0,005`;
- H2 deterministico: 3/3 start completati;
- reward aggregate: `151,862`;
- cicli: 2 su `-0,20`, 2 nominali, 3 su `+0,20`;
- nessun clipping.

### `LR=2e-6`

- KL RLlib: `0,003185`;
- KL empirico fisso: `0,002455`;
- `-0,20` e nominale completati;
- `+0,20` fallito a 215 step per penetrazione `25,144 mm`.

`LR=2e-6` e stato quindi scartato. `5e-6` non e stato eseguito perche il salto
piu piccolo aveva gia superato il margine dinamico. La fascia KL generica
`0,002-0,01` si e dimostrata troppo permissiva per questo sistema closed-loop.

## Breve catena PPO a `LR=1e-6`

La catena e stata estesa fino alla logical iteration 5, conservando ogni
checkpoint come rollback.

| Checkpoint | KL cumulativo da H0 | Gate eseguito | Risultato |
| --- | ---: | --- | --- |
| iter2 | 0,000548 | H2 tre start | 3/3, aggregate 151,862 |
| iter3 | 0,001862 | `-0,20` | 500 step, reward 40,250, 24,097 mm |
| iter4 | 0,004328 | `+0,20` | 500 step, reward 54,483, 24,136 mm |
| iter5 | 0,007856 | H2 tre start | 3/3, aggregate 147,331 |

Iter5 e rimasta deterministicamente valida, ma:

- reward on-policy del batch: `-4,779`, contro `5,457` di iter4;
- aggregate H2: `147,331`, inferiore a iter2 (`151,862`) e a H1-bis
  (`151,253`);
- non esiste quindi evidenza di un miglioramento cumulativo dopo iter2.

La catena e stata arrestata senza ridurre ulteriormente LR o sigma.

## La policy si e davvero allontanata dal warm start

Si. Iter5 rispetto a H1-bis produce, sullo start `-0,20`:

- azioni diverse di `0,156 rad` RMS;
- cinematica protesica diversa di `0,110 rad` RMS;
- reference servita diversa di `0,109 rad` RMS.

La distanza della cinematica dal prescribed resta elevata:

- `0,353 rad` RMS su `-0,20`;
- `0,373 rad` RMS sul nominale;
- `0,378 rad` RMS su `+0,20`.

Questi dati confutano l'ipotesi di una policy congelata o appiattita sul
prescribed. Piccoli delta dei pesi vengono amplificati dalla dinamica
closed-loop e generano traiettorie sostanzialmente differenti.

## Probe a `sigma=0,0075`

E stata creata una copia diagnostica di iter2 modificando esclusivamente le
bias logstd. La media dell'actor e rimasta identica (`max diff=0` su 500
osservazioni) e il save/reload e risultato esatto.

Sul nominale, tre seed hanno completato tutti l'episodio:

| Seed | Step | Cicli | Reward | Penetrazione |
| ---: | ---: | ---: | ---: | ---: |
| 123 | 500 | 3 | 67,806 | 23,026 mm |
| 124 | 500 | 2 | 46,685 | 24,004 mm |
| 125 | 500 | 2 | 55,964 | 23,672 mm |

- successo nominale: 3/3;
- reward media: `56,819`, deviazione standard `8,644`;
- RMS reale del rumore: `0,007327`;
- nessun clipping.

Sul seed 123 multi-start:

- `-0,20`: 500 step, 2 cicli, reward `41,086`, `22,592 mm`;
- `+0,20`: fallimento a 210 step, 1 ciclo, `25,166 mm`.

Il rumore mantenuto per 50 ms non ha risolto il problema: sullo start `+0,20`
ha fallito a 39 step, senza cicli, a `25,083 mm`. La fragilita non dipende
quindi dal solo contenuto ad alta frequenza; una perturbazione persistente e
risultata ancora piu dannosa.

## Test causale H0 contro il primo update PPO

Con stesso start `+0,20`, seed 123 e `sigma=0,005`:

| Actor | Step | Cicli | Reward | Penetrazione |
| --- | ---: | ---: | ---: | ---: |
| H0 | 500 | 3 | 57,108 | 24,559 mm |
| iter2, un update PPO | 214 | 0 | -35,194 | 25,172 mm |

Il rumore realizzato era corretto in entrambi i casi (`0,00484-0,00489` RMS).
Sui primi 214 step, iter2 differisce da H0 di:

- `0,0408` RMS nelle azioni;
- `0,0271 rad` RMS nella cinematica protesica;
- `0,0269 rad` RMS nella reference servita.

H0 portato a `sigma=0,0075` fallisce invece a 210 step e `25,180 mm`.

La diagnosi e quindi doppia:

1. H0 possiede gia, sullo start `+0,20`, un limite di robustezza compreso tra
   `sigma=0,005` e `0,0075`.
2. Il primo update PPO restringe ulteriormente il recovery basin e rende non
   sostenibile persino `sigma=0,005` per lo stesso seed.

## Conclusione operativa

Il problema non e mancanza di learning. `LR=1e-6` modifica realmente la policy
e genera traiettorie ex novo differenti. Il problema e che il criterio di
selezione deterministico premia un piccolo guadagno medio mentre non rileva la
perdita di robustezza stocastica su uno start specifico.

Nessuno dei checkpoint actor aggiornati viene promosso come nuovo H0. Il
checkpoint canonico resta:

`validation/critic_warmup/2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last`

Questa scelta non riporta il progetto al prescribed: mantiene il warm start
imitativo come prior, actor deployable a 35 feature, critic privilegiato e
reward ex novo. Evita soltanto di accettare un update PPO che supera H2
deterministico ma fallisce il gate stocastico.

## TODO vincolanti

- Ripartire da H0, non da iter2-iter5.
- Inserire nel gate di ogni update almeno `+0,20`, seed 123, stocastico a
  `sigma=0,005`, oltre ai tre rollout deterministici.
- Bilanciare esattamente il campionamento dei tre start. Con 13 EnvRunner sono
  stati raccolti `1576/1260/1260` step per update; usare un multiplo di tre o
  una riduzione esplicita per start.
- Analizzare return e advantage separatamente per start, evitando che la media
  globale accetti un update che sacrifica il worst case.
- Solo dopo il superamento del gate a `0,005`, riprovare `0,0075` su almeno tre
  seed per ciascuno dei tre start.
- Non modificare reward, soglia di 25 mm o informazioni dell'actor per far
  passare il test. Se nessun update supera il gate, intervenire sul protocollo
  robusto di ottimizzazione o sui dati on-policy di recovery, mantenendo il
  prescribed fuori dall'actor e dalla reward ex novo.

## File modificati

- `Trajectory Generator/baseline_MLP/configure_actor_exploration.py`
- `validation/compare_policy_checkpoints.py`
- `validation/compare_rollout_traces.py`
- `validation/analyze_stochastic_rollouts.py`
- `validation/test_actor_exploration_configuration.py`
- `validation/test_compare_policy_checkpoints.py`
- `validation/test_compare_rollout_traces.py`
- `validation/test_analyze_stochastic_rollouts.py`

## Verifiche

- test focalizzati dei configuratori e analizzatori;
- audit esatto dei parametri della media durante la modifica del sigma;
- audit dell'optimizer Adam e del learning rate dopo ogni restore;
- H2 deterministico sui tre start;
- probe stocastici multi-seed e multi-start;
- confronto closed-loop H0 -> iter2 con stesso seed e sigma.
