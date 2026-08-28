# Diagnostica rumore sul teacher e correlazione temporale dell'actor

Data: 2026-07-12

## Problema

Il warm start DAgger round 2 produceva una media deterministica utile, ma la
validazione `training-like` richiedeva una deviazione standard molto bassa:
`sigma=0.003`. Con rumore bianco indipendente a ogni step, questo candidato
chiudeva almeno un ciclo in 2 seed su 3, mentre `sigma=0.005` e `0.01`
fallivano intorno a 200 step.

Questo lasciava due ipotesi non discriminate:

1. la dinamica, la FSM o le soglie della reward sono intrinsecamente
   incompatibili con perturbazioni superiori a `0.003`;
2. il teacher rimane robusto, ma l'actor imitativo amplifica piccoli errori
   closed-loop e quindi nasconde una propria fragilita' dietro un sigma basso.

Era inoltre possibile che il rumore bianco a 100 Hz fosse artificialmente
severo rispetto a perturbazioni con la stessa scala ma coerenti per 50-100 ms.

## Strategia

Sono stati eseguiti due test discriminanti senza avviare H1 e senza aggiornare
i pesi dell'actor.

### Test A: sweep del teacher prescribed

Il teacher full-episode e' stato eseguito nello stesso contratto target del
warm start, aggiungendo rumore gaussiano bianco alle azioni realmente servite:

```text
sigma                  0.003, 0.005, 0.010
seed                   123
durata episodio        500 step / 5 s
periodo del rumore     1 step / 10 ms
soglie penetrazione    15 mm soft / 25 mm hard
FSM e limiter          invariati
```

Le label pulite del teacher restano separate dalle azioni rumorose eseguite,
in modo che il probe non modifichi semanticamente il dataset imitativo.

### Test B: rumore mantenuto sull'actor

Per l'actor e' stata aggiunta una modalita' diagnostica che ricalcola media e
deviazione standard della policy a ogni osservazione, ma mantiene lo stesso
draw normale standard per 5 o 10 step:

```text
hold 5 step            50 ms
hold 10 step           100 ms
sigma nominale         invariato rispetto al checkpoint
```

Sono stati scelti i seed che fallivano con rumore bianco:

```text
sigma 0.003             seed 125
sigma 0.005             seed 123
sigma 0.010             seed 123
```

Il confronto e' a parita' di sigma nominale. L'RMS effettivamente realizzato
e' registrato per entrambe le azioni, per rendere visibile la normale
variabilita' campionaria dovuta al numero ridotto di draw nei segnali held.

## Risultati del teacher

Il baseline prescribed pulito era:

```text
rumore   step   return    HS/TO/cicli   max pen    fine                 gate
0        500    +64.032      3/3/2      22.944 mm  episode_time_limit   PASS
```

Lo sweep bianco ha prodotto:

```text
sigma    RMS knee/ankle       step   return    HS/TO/cicli   max pen    fine                  gate
0.003    .003095/.002932      500    +64.302      3/3/2      22.976 mm  episode_time_limit    PASS
0.005    .005158/.004887      500    +64.249      3/3/2      22.998 mm  episode_time_limit    PASS
0.010    .010481/.009817      388    +16.724      2/2/1      23.061 mm  phase_timeout:swing   FAIL
```

Il teacher e' quindi pienamente robusto a `sigma=0.005` sotto la stessa
dinamica, FSM, reward e soglia da 25 mm. A `sigma=0.01` esiste invece anche un
limite del sistema: il teacher chiude un ciclo ma non completa l'episodio per
timeout di swing. Non termina per penetrazione.

## Risultati dell'actor

Confronto sui seed fragili:

```text
sigma   rumore          RMS knee/ankle       step   return    HS/TO/cicli   max pen    fine
0.003   white PPO       circa .003/.003       210   -17.892      1/1/0      25.765 mm  penetrazione
0.003   held 50 ms      .002801/.003031       359   +32.269      2/2/1      25.587 mm  penetrazione
0.003   held 100 ms     .003091/.003299       221   -59.565      1/0/0      22.080 mm  stance timeout

0.005   white PPO       circa .005/.005       206   -11.581      1/1/0      25.387 mm  penetrazione
0.005   held 50 ms      .004854/.004267       204   -18.472      1/1/0      25.330 mm  penetrazione
0.005   held 100 ms     .005202/.004645       206   -14.500      1/1/0      25.797 mm  penetrazione

0.010   white PPO       circa .010/.010       203   -12.915      1/1/0      25.391 mm  penetrazione
0.010   held 50 ms      .009854/.008367       214   -21.959      1/1/0      25.629 mm  penetrazione
0.010   held 100 ms     .010404/.009291       206   -14.124      1/1/0      25.771 mm  penetrazione
```

Il rumore a 50 ms recupera completamente il seed 125 a `sigma=0.003`: da 210
a 359 step e da zero a un ciclo valido. Questo conferma che il dithering
indipendente a 100 Hz era una parte reale del failure.

Il recupero non si estende a `sigma=0.005` o `0.01`. A queste ampiezze white,
50 ms e 100 ms falliscono tutti nella stessa regione di circa 200-214 step.
Inoltre 100 ms e' troppo persistente anche a `sigma=0.003` e impedisce il TO.

## Interpretazione

I risultati separano le responsabilita'.

- **Reward/FSM/dinamica non spiegano da sole il limite a 0.005.** Il teacher
  completa 500 step e due cicli con `sigma=0.005`; l'actor termina a 204-206
  step con ogni struttura temporale provata.
- **L'actor ha una fragilita' closed-loop reale.** La media imitativa funziona
  vicino alla traiettoria nominale, ma non corregge in modo robusto stati
  perturbati abbastanza da tollerare `sigma>=0.005`.
- **Il rumore bianco a 100 Hz e' parzialmente patologico, non l'unica causa.**
  Hold 50 ms salva il seed fragile a `0.003`, ma non consente di aumentare
  l'ampiezza esplorativa.
- **Anche il sistema ha un limite superiore.** Il teacher a `0.01` fallisce
  per timeout di swing, sebbene molto piu' tardi dell'actor e dopo un ciclo.
- **Il precedente gate relativo era troppo permissivo per dichiarare robusta
  l'esplorazione.** `sigma=0.003` resta eseguibile, ma non dimostra una policy
  capace di esplorare ampiamente attorno al prior prescribed.

La modalita' held e' solo diagnostica. Non va inserita direttamente nel PPO
standard: PPO assume la probabilita' delle azioni campionate dalla propria
distribuzione condizionata allo stato corrente. Una correlazione esterna deve
essere modellata esplicitamente nella policy/distribuzione o nel contratto
dell'ambiente, altrimenti log-probability e ratio PPO non descrivono il processo
che ha generato le azioni.

## Decisione operativa

Non e' corretto aumentare semplicemente sigma e non e' corretto considerare
`0.003` una soluzione definitiva. Il candidato puo' essere usato solo come
bootstrap conservativo, dichiarando che l'esplorazione e' limitata.

Prima di un training lungo, il passo consigliato e' un protocollo breve:

1. H1 singola con `sigma=0.003`, `freeze_logstd=true` e tutti i gate gia'
   predisposti, per misurare se un solo update migliora o degrada la media;
2. H2 deterministico e stocastico immediato, senza promuovere automaticamente
   il checkpoint;
3. se H1 non aumenta la robustezza, lavorare sulla recovery closed-loop
   dell'actor o su una distribuzione temporale correttamente modellata, invece
   di abbassare ulteriormente sigma;
4. obiettivo minimo di robustificazione: rendere l'actor capace di superare un
   ciclo a `sigma=0.005`, livello che il teacher ha dimostrato sostenibile.

H1 non e' stato eseguito in questa diagnostica.

## File modificati

```text
Trajectory Generator/baseline_MLP/exploration_noise.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/target_domain_imitation.py
Trajectory Generator/baseline_MLP/README.md
validation/test_exploration_noise.py
validation/test_rollout_eval.py
```

## Verifiche

```text
unittest discover             50/50 PASS
py_compile                    PASS
git diff --check              PASS
runtime smoke held 5 step     PASS
teacher full-episode sweep    completato
actor held 50/100 ms          completato
```

## Artefatti

```text
validation/teacher_noise_runs/
validation/exploration_noise_runs/
```

## TODO

- [x] Eseguire sweep white `0.003/0.005/0.01` sul teacher full-episode.
- [x] Eseguire rumore held 50 e 100 ms sull'actor a sigma invariato.
- [x] Registrare sigma nominale, RMS realizzato, eventi, penetrazione e causa
      di terminazione.
- [x] Eseguire H1 solo come probe singolo controllato, non come training lungo.
- [x] Richiedere al checkpoint post-H1 un miglioramento di robustezza misurabile
      prima di aumentare sigma o proseguire con altre iterazioni. Il gate e'
      fallito e non e' stata eseguita l'iterazione 2; vedere
      `2026-07-12_h1_single_iteration_h2_rejected.md`.
