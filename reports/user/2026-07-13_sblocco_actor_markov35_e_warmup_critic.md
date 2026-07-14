# Sblocco actor Markov 35-feature e warm-up del critic

## Problema

La validazione sembrava indicare che reintrodurre lo stato interno del controller
nell'actor rendesse la policy fragile. Il candidato 35-feature precedente
falliva gli start temporali alternativi dopo circa 50 step, mentre il contratto
25-feature completava almeno lo start positivo. Questo risultato era stato
interpretato troppo presto come evidenza contro l'architettura 35-feature.

L'obiettivo corretto era distinguere tre possibilita':

1. lo stato aggiunto non e' utile o e' intrinsecamente dannoso;
2. reward o soglia di penetrazione bloccano una policy altrimenti valida;
3. il protocollo di adattamento supervisionato insegna una mappatura incoerente.

## Diagnosi della causa

E' stato aggiunto `validation/diagnose_markov_state_adaptation.py` per analizzare
le dieci feature aggiunte, il contributo al primo layer, le azioni prodotte e la
coerenza tra osservazione visitata e label teacher.

La causa principale era nel dataset DAgger usato per l'adattamento:

```text
campioni recovery originali          1500
campioni ancora allineati a FSM       356
campioni con label fuori fase        1144
frazione fuori fase                  76.27%
primo mismatch nei tre trace       120/119/120
```

Le azioni teacher erano associate per indice temporale fisso. Dopo che il
rollout perturbato cambiava contatto o stato FSM, la label apparteneva a una
fase discreta diversa. La rete riceveva quindi supervisione contraddittoria.

Sono emersi anche due problemi secondari:

- le accelerazioni del reference servito arrivavano fino a 60 rad/s2 e
  dominavano numericamente gli input di posizione;
- il dataset precedente era fortemente sbilanciato verso il nominale e
  l'aggiornamento delle sole nuove colonne non poteva apprendere le interazioni
  con FSM, contatto e stato fisico.

La reward non era la causa del blocco. Il guard a 25 mm stava correttamente
rivelando l'errore della policy; non e' stato modificato.

## Correzioni

In `target_domain_markov_adaptation.py` sono state introdotte:

- troncamento automatico di ogni trace al primo mismatch discreto;
- scale fisiche fisse per velocita' e accelerazioni delle nuove feature;
- peso indipendente per i dataset degli start alternativi;
- opzione controllata per adattare l'intera mean-network;
- `logstd` sempre congelato e critic escluso dall'adattamento.

In `target_domain_imitation.py` l'ottimizzazione puo' usare input scalati senza
cambiare il contratto runtime: dopo il fit le scale vengono assorbite nei pesi
del primo layer, quindi il modulo salvato continua a ricevere osservazioni
fisiche non normalizzate.

Il dataset finale contiene:

```text
ancore nominali del source actor     16000
recovery stocastici phase-aligned      712
teacher su start -0.20/+0.20 s        8000
totale                               24712
```

L'intera rete che produce la media e' stata adattata, mentre la testa `logstd`
e' rimasta bit-identica. Non sono stati eseguiti update PPO e il critic non e'
stato addestrato in questa fase.

## Risultato actor

Checkpoint selezionato:

```text
Trajectory Generator/runs/training/
target_domain_markov35_phase_aligned_scaled_full_r32_alt8_2026-07-13/
rl_module_target_adapted
```

Digest actor:

```text
a0801a9e635db4f2973da7d8f6461cbbf7b1643efef1dedc2baafd9c9f95ca21
```

Metriche offline:

```text
RMSE aggregato prima                  0.019657
RMSE aggregato dopo                   0.008144
RMSE start -0.20 s                    0.007830
RMSE start +0.20 s                    0.007209
shift nominale RMS                    0.004175
shift nominale massimo                0.031594
```

Il source produceva sul primo step dello start `-0.20 s` un comando ginocchio
`0.807` contro target `0.206`; il candidato produce `0.219`.

Validazione deterministica completa:

```text
start          step  return   cicli  max pen    clipping
nominale        500   51.797     2    23.960 mm    0
-0.20 s         500   35.788     2    24.602 mm    0
+0.20 s         500   55.926     2    24.324 mm    0
```

Validazione stocastica con `sigma=0.005`:

```text
seed           step  return   cicli  max pen    clipping
123             500   62.145     3    23.268 mm    0
124             500   54.961     3    23.884 mm    0
125             500   53.981     3    24.165 mm    0
```

Il candidato passa 3/3 start deterministici e 3/3 seed stocastici.

## Refinement rifiutato

Un secondo adattamento con il doppio delle ancore nominali e maggiore
regolarizzazione ha ridotto lo shift RMS nominale a `0.002948`, ma lo start
`-0.20 s` e' terminato allo step 221 con penetrazione `25.090 mm`.

Il candidato e' stato rifiutato. Questo dimostra che minimizzare ulteriormente
la singola metrica offline non e' monotono rispetto alla robustezza dinamica;
non e' stata alzata la soglia di sicurezza.

## Contratto selezionato

`training_exnovo_cfg.yaml` ora fissa:

```text
feature actor                          35
stato controller actor-visible        true
diagnostiche controller actor-visible false
prescribed actor-visible              false
critic privilegiato                   true
gait clock                            false
sigma                                 0.005
soglie penetrazione                   15/25 mm
```

Le dieci feature aggiunte descrivono esclusivamente stato disponibile a runtime:
previous endpoint, reference servita con derivate e ultimo comando SEA per
ginocchio e caviglia. Non contengono prescribed futuro.

## Warm-up critic

Il critic e' stato riscaldato per una iterazione con actor congelato:

```text
step campionati                       4096
EnvRunner                               13
epoch                                  10
learning rate                        1e-4
vf loss                           0.558768
vf explained variance             0.238068
mean KL                               0.0
actor max change                       0.0
```

Digest:

```text
actor prima/dopo       a0801a9e...ca21, identico
critic prima           5ce4ef41...1096
critic dopo            4584399f...f6e
critic dopo restore    4584399f...f6e
```

Checkpoint completo da usare per H1:

```text
validation/critic_warmup/
2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last
```

Il primo tentativo di warm-up e' stato bloccato dalla sandbox macOS prima di
inizializzare Ray. Il retry fuori sandbox e' quello valido riportato sopra.

L'audit ha inoltre rilevato un errore di sola provenance nel parser: quando era
fornito un `--warm-start-source` custom senza `--warm-start-source-config`, il
report indicava lo YAML dell'official checkpoint invece del file adiacente al
source effettivo. Il manifest adiacente 35-feature e il digest actor erano gia'
usati e validati correttamente, quindi pesi e comportamento del warm-up non sono
cambiati. Il parser ora lascia che `warm_start.py` risolva la configurazione
adiacente corretta:

```text
Trajectory Generator/runs/training/
target_domain_markov35_phase_aligned_scaled_full_r32_alt8_2026-07-13/
training_cfg.resolved.yaml
```

## File modificati

```text
Trajectory Generator/baseline_MLP/target_domain_imitation.py
Trajectory Generator/baseline_MLP/target_domain_markov_adaptation.py
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
validation/diagnose_markov_state_adaptation.py
validation/validate_training_config.py
validation/controller_memory_ablation/2026-07-13_markov35_final_gate.json
reports/user/2026-07-13_sblocco_actor_markov35_e_warmup_critic.md
```

## Verifiche

```text
py_compile file modificati            PASS
unit test                              PASS, 58/58
config smoke                           PASS
git diff --check                       PASS
save/reload actor adaptation           PASS, bit-exact
actor dopo critic warm-up              PASS, bit-exact
restore checkpoint critic              PASS, digest identico
```

## Limiti e prossimo passo

Il risultato valida un warm start, non una policy clinica finale. I test
coprono AB06 e tre start dello stesso trial; non dimostrano generalizzazione a
soggetti, trial o velocita' diverse. Il nominal return e' inferiore a quello del
baseline 25-feature e il batch critic-only contiene una terminazione per
penetrazione: entrambi vanno monitorati durante H1.

Il prossimo passo corretto e' riprendere dal checkpoint completo con
`freeze_actor=false`, eseguire una sola H1 controllata e applicare subito H2.
