# H1 Markov35 dopo warm-up critic e H2 respinta

## Obiettivo

Eseguire il primo aggiornamento PPO controllato dell'actor Markov a 35 feature,
ripartendo dal checkpoint completo ottenuto dopo il warm-up del critic. Il
protocollo richiedeva una sola iterazione con actor sbloccato, `sigma=0.005` e
`logstd` congelato, seguita immediatamente da H2.

Reward, FSM, limiter, contratto osservativo e soglie di penetrazione `15/25 mm`
non sono stati modificati.

## H1 eseguita

Checkpoint di partenza:

```text
validation/critic_warmup/
2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last
```

Il resume ha ripristinato la logical iteration 1. E' stata eseguita soltanto la
logical iteration 2, quindi un unico update PPO con actor attivo:

```text
freeze_actor                         false
freeze_logstd                         true
sigma knee/ankle                     0.005 / 0.005
batch                                  4096 step
epoch                                    10
learning rate                          1e-4
EnvRunner                                 13
```

Esito del batch H1:

```text
episode return mean                 -32.5582
episode length mean                  229.000
terminazioni per penetrazione               2
policy loss                           0.07738
value loss                            0.549997
value explained variance      0.238068 -> 0.455200
mean KL                               0.154690
KL target RLlib                       0.010000
```

Il critic e' migliorato ed e' cambiato realmente:

```text
critic prima    4584399f63781c48002f8169240ae686b0317aaa22bab9799d553d7fbc2a9f6e
critic dopo     a4c72e333e4843799fc8beeb9d27a671761e209304bfa8990fe32e7d82562091
```

Il KL e' pero' pari a circa `15.5` volte il target. Il clipping PPO non ha
quindi mantenuto il primo update vicino alla policy iniziale dopo dieci epoch.

## Audit actor e sigma

```text
actor H0 digest      a0801a9e635db4f2973da7d8f6461cbbf7b1643efef1dedc2baafd9c9f95ca21
actor H1 digest      266f43bdbca6ff7db0dce2129041615a1cb019e0f65516ec2dea2ac3729c6a3a
max variazione parametro                          0.001716
RMSE media su 500 osservazioni nominali            0.001637
max spostamento media                              0.005132
```

Lo spostamento massimo della media equivale a circa una sigma. Le righe di
peso e bias della testa `logstd` sono bit-identiche, l'output `logstd` non e'
cambiato e la sigma media resta `0.005000019` su entrambi gli attuatori. La
regressione non dipende quindi da una modifica accidentale dell'esplorazione.

## H2 deterministica

```text
start       step H0->H1   return H0->H1      cicli H0->H1   pen H0->H1
nominale    500 -> 500     51.798 -> 47.730      2 -> 2      23.960 -> 24.151 mm
-0.20 s     500 -> 231     35.788 -> 15.447      2 -> 1      24.602 -> 25.334 mm
+0.20 s     500 -> 500     55.926 -> 57.354      2 -> 2      24.324 -> 23.922 mm
```

Lo start `-0.20 s` termina per penetrazione allo step 231. Il guard non e'
stato alzato. Questo singolo risultato e' sufficiente a respingere H1.

## H2 stocastica

Rollout nominali con `sigma=0.005`:

```text
seed        step H0->H1   return H0->H1      cicli H0->H1   pen H0->H1
123         500 -> 500     62.145 -> 61.820      3 -> 2      23.268 -> 22.934 mm
124         500 -> 500     54.961 -> 40.804      3 -> 2      23.884 -> 24.945 mm
125         500 -> 500     53.981 -> 61.042      3 -> 3      24.165 -> 23.880 mm
```

Tutti i seed completano l'episodio e non presentano clipping. Il seed 124 ha
pero' soltanto `0.055 mm` di margine dal guard e perde `14.157` punti di return.
La stochastic nominale non compensa il fallimento sullo start alternativo.

## Interpretazione

Il warm-up del critic e' stato utile: l'explained variance e' salita e il nuovo
H1 conserva il nominale, mentre la precedente H1 collassava anche li'. Non e'
pero' sufficiente per rendere sicuro l'update actor standard.

La causa immediata e' doppia:

1. dieci epoch a `1e-4` spostano la distribuzione molto oltre il KL target;
2. il batch PPO parte sempre dallo start nominale e non protegge la regione di
   stato `-0.20 s` che l'adattamento imitativo aveva reso robusta.

Un piccolo spostamento medio della rete condivisa puo' quindi migliorare uno
start e peggiorarne un altro. Questo non respinge l'actor a 35 feature e non
indica leakage prescribed: respinge lo specifico update PPO H1.

## Decisione

```text
stato H1                              REJECT_H1
proseguire da checkpoint H1           no
promuovere rl_module_best/last         no
checkpoint valido                     H0 con critic warm-up
alzare guard                           no
modificare reward sulla base di H1     no
```

Il prossimo H1-bis deve partire nuovamente dal checkpoint H0, includere
copertura esplicita degli start iniziali e applicare un budget actor/KL molto
piu' piccolo. Prima di un training multi-iterazione dovra' superare lo stesso
gate H2.

## Artefatti

```text
training H1:
Trajectory Generator/runs/training/validation/warm_start_h1_runs/
2026-07-13_markov35_after_critic_warmup_iter2/

H2:
validation/warm_start_h2_runs/2026-07-13_markov35_post_h1_*/

gate:
validation/warm_start_h1_runs/
2026-07-13_markov35_after_critic_warmup_h1_h2_gate.json
```

## Verifiche

```text
resume full checkpoint                 PASS
una sola iterazione actor-enabled      PASS
critic aggiornato                      PASS
logstd e sigma bit-identici            PASS
H2 seriale, 3 start deterministici     PASS esecuzione, FAIL gate
H2 seriale, 3 seed stocastici          PASS esecuzione
action clipping                        0 in 6/6 rollout
processi training/rollout/EnvRunner    nessuno, verificato
```

## TODO

- [ ] Progettare H1-bis con copertura esplicita degli start nominale e `+/-0.20 s`.
- [ ] Ridurre e rendere verificabile il budget KL del primo update actor.
- [ ] Ripetere H2 completa prima di qualsiasi training lungo.
