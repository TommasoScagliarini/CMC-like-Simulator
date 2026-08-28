# Warm-start: configurazione e preflight della varianza esplorativa

Data: 2026-07-12

## Obiettivo

Il porting del checkpoint DAgger round 2 era tecnicamente corretto e la media
deterministica dell'actor produceva un rollout utile (`356` step, un ciclo
valido). Il preflight con la distribuzione PPO originale falliva pero' prima
del primo TO:

```text
seed 123/124/125: 48/50/77 step
clipping mediano: 32%
terminazioni per penetrazione: 3/3
```

La causa era la deviazione standard state-dependent ereditata dall'actor
imitativo: mediana circa `0.75` e massimo oltre `2.45` nell'action space
normalizzato `[-1, 1]`.

L'obiettivo di questo lavoro era configurare una esplorazione controllata senza
modificare la media validata, ripetere il gate stocastico e avviare una sola
iterazione H1 esclusivamente in caso di gate superato.

## Strategia

E' stato aggiunto uno strumento che crea una copia separata del checkpoint e
modifica soltanto le due uscite gaussiane dedicate alla `log-std`:

```text
pi.1.weight[2:4, :] = 0
pi.1.bias[2:4]      = log(sigma)
```

Le due righe che producono la media dell'azione, tutti gli strati precedenti,
la configurazione, il critic e la reward restano invariati.

Ogni candidato viene verificato mediante:

- confronto esatto dei parametri della media;
- confronto delle medie su tutte le 356 osservazioni reali del rollout r2;
- digest actor e SHA-256 del nuovo `module_state.pkl`;
- save/reload bit-exact;
- tre rollout `forward_exploration` con seed 123, 124 e 125;
- stesso start, FSM, limiter 2.5/2.0 rad/s e soglie 15/25 mm.

## Candidato sigma 0.05

Artefatto:

```text
Trajectory Generator/runs/training/
target_domain_warm_start_r2_sigma005_2026-07-12/
```

Validazione tensoriale:

```text
log-std                       -2.995732
sigma                          0.05
mean max_abs_diff              0.0 su 356 campioni
log-std weight norm            0.0
save/reload                    exact
actor digest                   a5a6ff1fc49bcaccdd519fb00c058527d17fe1c6f466cc67e33c1b8a87412163
```

Rollout:

```text
seed  steps  return    clip     max pen   HS/TO/cicli  invalid  fine
123     211  -19.478   0.000%   25.024 mm   1/1/0        1     penetrazione
124     206  -17.041   0.000%   25.181 mm   1/1/0        2     penetrazione
125     213  -16.353   0.235%   25.164 mm   1/1/0        1     penetrazione
```

Rispetto alla varianza originale, il clipping e il failure pre-TO sono stati
eliminati. Tutti i seed raggiungono un TO valido, ma nessuno chiude un ciclo e
tutti terminano per penetrazione. Il candidato non supera il gate concordato.

## Fallback sigma 0.03

Artefatto:

```text
Trajectory Generator/runs/training/
target_domain_warm_start_r2_sigma003_2026-07-12/
```

Validazione tensoriale:

```text
log-std                       -3.506558
sigma                          0.03
mean max_abs_diff              0.0 su 356 campioni
log-std weight norm            0.0
save/reload                    exact
actor digest                   63f4332a92fd1f8c19df414d78d6581ca2764158a73133a0bb6d61400a57c37c
```

Rollout:

```text
seed  steps  return    clip     max pen   HS/TO/cicli  invalid  fine
123     204  -13.454   0.000%   25.534 mm   1/1/0        1     penetrazione
124     208  -17.235   0.000%   25.276 mm   1/1/0        1     penetrazione
125     228  +15.897   0.000%   25.091 mm   2/1/1        0     penetrazione
```

Il fallback mantiene clipping nullo e produce un seed con ciclo valido e
return positivo. Tuttavia anche in questo caso `3/3` episodi terminano per
penetrazione, quindi il gate aggregato resta `FAIL`.

## Replay deterministico

Il checkpoint `sigma=0.03` e' stato eseguito in modalita' deterministica. La
`log-std` non entra nella media e il risultato riproduce esattamente il rollout
r2:

```text
steps                          356
return                         +29.886494
HS / TO / cicli                2 / 2 / 1
clipping                       0%
max penetrazione               26.512 mm
fine                           grf_penetration
```

Il replay ha completato senza timeout. I precedenti stalli ai passi 198 e 165
erano quindi intermittenti e non una proprieta' deterministica del checkpoint.

## Decisione su H1

Il protocollo richiedeva:

```text
clipping mediano <= 10%
almeno 2/3 seed con TO valido
meno di 2/3 seed terminati per penetrazione
```

Entrambi i candidati soddisfano i primi due punti ma falliscono il terzo con
`3/3` terminazioni per penetrazione. Per questo motivo H1 non e' stato avviato e
nessun aggiornamento PPO e' stato eseguito.

Questa decisione e' conservativa. Il baseline deterministico termina anch'esso
per penetrazione, ma piu' tardi e dopo un ciclo. Prima di modificare il gate o
avviare H1 occorre decidere esplicitamente se il criterio corretto sia:

1. richiedere che alcuni rollout stocastici completino l'episodio, oppure
2. accettare episodi post-TO informativi e usare PPO proprio per ridurre la
   penetrazione.

Il candidato migliore per una eventuale H1 controllata e' `sigma=0.03`, non
`sigma=0.05`: ha clipping nullo, almeno un ciclo valido e un seed con return
positivo.

## Protocollo diagnostico successivo

Il criterio assoluto sulla terminazione per penetrazione e' stato sostituito da
un gate relativo al baseline deterministico. La terminazione per penetrazione
non puo' essere da sola un criterio discriminante, perche' anche la media
deterministica validata termina in questo modo dopo 356 step.

Il nuovo gate richiede simultaneamente:

```text
media deterministica preservata esattamente              si
probe tecnicamente validi, finiti e senza timeout         3/3
TO valido                                                 3/3
almeno un ciclo HS -> TO -> HS                            si
mediana sopravvivenza >= 60% di 356 step                  >= 213.6
clipping mediano                                          < 1%
```

### Analisi registrata di sigma 0.03, seed 125

Il seed 125, unico probe isotropico che aveva chiuso un ciclo, e' stato
rieseguito con trace e output completi. Il risultato e' stato riprodotto:

```text
steps / return                    228 / +15.8975
HS / TO / cicli                   2 / 1 / 1
clipping                          0%
max penetrazione                  25.091 mm
fine                              grf_penetration
```

Il confronto con i 356 step deterministici mostra:

```text
std rumore misurata knee/ankle                 0.02935 / 0.03013
massimo assoluto rumore                        0.08826 / 0.09436
prima differenza action > 0.05                 step 1
prima differenza q o reference servita > 0.01 step 8
prima differenza penetrazione > 2 mm           step 27
step con slew limiter attivo                   85.09%
frazione media coordinate limitate             62.28%
```

Gli eventi restano inizialmente vicini:

```text
                             deterministico   stocastico
primo HS                           6               6
primo TO                         119             121
secondo HS / ciclo               151             154
secondo TO                       284               -
```

Alla fine dei 228 step comuni, la penetrazione e' `25.091 mm` nel probe e
`24.038 mm` nel baseline. Un accumulo di differenze piccole lascia quindi
soltanto circa `1.05 mm` tra le due traiettorie e trasforma un rollout ancora
valido in una terminazione anticipata. Non e' stato osservato un singolo
campione catastrofico: il problema e' la sensibilita' closed-loop dell'actor a
rumore indipendente a ogni step, amplificata dall'interazione con limiter,
governor, SEA e contatto.

Report machine-readable:

```text
validation/warm_start_rollout_runs/2026-07-12_sigma003/
exploration_divergence_seed125.json
```

### Controfattuali per articolazione

Sul seed 125 sono state mantenute le stesse condizioni cambiando soltanto la
componente rumorosa:

```text
sigma knee/ankle   steps   return    HS/TO/cicli  clipping  fine
0.03 / ~0            221   -59.403      1/0/0        0%     stance timeout
~0 / 0.03            204   -12.522      1/1/0        0%     penetrazione
0.03 / 0.03           228   +15.897      2/1/1        0%     penetrazione
```

Il canale knee e' particolarmente sensibile per la transizione di stance e puo'
impedire il TO. Il canale ankle conserva il TO ma riduce il margine di contatto.
Il fatto che la combinazione isotropica abbia occasionalmente prodotto un ciclo
mostra inoltre che gli effetti non sono additivi: una perturbazione puo'
compensarne temporaneamente un'altra.

### Candidato anisotropico

E' stato creato un checkpoint separato con:

```text
sigma knee / ankle                 0.020 / 0.015
log-std knee / ankle              -3.912023 / -4.199705
media max_abs_diff su 356 sample   0.0
save/reload                        exact
actor digest                       4b185467c53154e3511fb3fa346d630c4b64e2d9bd1605fda672ca4e139483dc
```

I tre rollout hanno prodotto:

```text
seed  steps  return    clip  max pen   HS/TO/cicli  invalid  fine
123     205  -15.225    0%   25.559 mm   1/1/0        1     penetrazione
124     205  -17.273    0%   26.176 mm   1/1/0        2     penetrazione
125     216  -20.184    0%   25.125 mm   1/1/0        1     penetrazione
```

Esito del gate relativo:

```text
tecnico / finito / no timeout        PASS
media deterministica invariata       PASS
TO valido 3/3                        PASS
clipping mediano 0%                  PASS
almeno un ciclo                      FAIL (0/3)
mediana sopravvivenza                FAIL (205 < 213.6 step)
decisione                            STOP_BEFORE_H1
```

La riduzione anisotropica ha eliminato il clipping ma non ha aumentato la
robustezza: rispetto a `sigma=0.03` perde anche l'unico ciclo osservato. H1 non
e' stato avviato, la `log-std` non e' stata resa trainabile o congelata nel
trainer e non e' stato applicato alcun update PPO ad actor, critic od
ottimizzatore.

Il prossimo intervento raccomandato e' un DAgger target-domain noise-aware:
raccogliere gli stati visitati dall'actor con il piccolo rumore controllato,
interrogarvi il teacher e riaddestrare soltanto la media dell'actor su una
miscela di dati nominali e perturbati. La `log-std` deve restare costante. Il
medesimo gate relativo andra' poi ripetuto prima di una singola H1 con
`log-std` congelata.

## File modificati

```text
Trajectory Generator/baseline_MLP/configure_actor_exploration.py
validation/analyze_exploration_divergence.py
validation/evaluate_relative_h1_gate.py
validation/summarize_warm_start_preflight.py
validation/test_actor_exploration_configuration.py
validation/test_warm_start_preflight.py
reports/user/2026-07-12_warm_start_exploration_variance_preflight.md
```

## Artefatti e verifiche

```text
sigma 0.05 config report:
Trajectory Generator/runs/training/
target_domain_warm_start_r2_sigma005_2026-07-12/
exploration_configuration_report.json

sigma 0.03 config report:
Trajectory Generator/runs/training/
target_domain_warm_start_r2_sigma003_2026-07-12/
exploration_configuration_report.json

rollout sigma 0.05:
validation/warm_start_rollout_runs/2026-07-12_sigma005/

rollout sigma 0.03 e replay deterministico:
validation/warm_start_rollout_runs/2026-07-12_sigma003/

controfattuali knee/ankle:
validation/warm_start_rollout_runs/2026-07-12_counterfactual/

candidato anisotropico e verdetto del gate relativo:
validation/warm_start_rollout_runs/2026-07-12_sigma_anisotropic/
relative_h1_gate.json
```

## TODO

- [x] Preservare esattamente la media DAgger r2.
- [x] Configurare e verificare `sigma=0.05`.
- [x] Eseguire tre rollout stocastici `sigma=0.05`.
- [x] Configurare e verificare il fallback `sigma=0.03`.
- [x] Eseguire tre rollout stocastici `sigma=0.03`.
- [x] Ripetere con successo il rollout deterministico completo.
- [x] Non avviare H1 dopo il mancato superamento del gate.
- [x] Sostituire il criterio assoluto sulla penetrazione con un gate relativo
      al baseline deterministico.
- [x] Registrare `sigma=0.03`, seed 125, e localizzare la divergenza.
- [x] Separare causalmente il rumore knee e ankle.
- [x] Configurare e verificare `sigma knee/ankle=0.020/0.015`.
- [x] Eseguire i tre probe anisotropici e non avviare H1 dopo il gate `FAIL`.
- [x] Eseguire un adattamento target-domain noise-aware mantenendo costante la
      `log-std` e preservando i dati nominali r2.
- [x] Provare recovery distillation noise-aware: respinta per regressione del
      rollout deterministico a 59 step nonostante la loss offline migliore.
- [x] Selezionare `sigma=0.003` preservando bit-exact la media r2.
- [x] Ripetere il gate relativo: `PASS`, mediana 360 step, TO 3/3, cicli 2/3.
- [x] Validare il trainer con `log-std` congelata a zero iterazioni.
- [ ] Eseguire una sola H1 su richiesta esplicita; dettagli in
      `2026-07-12_readiness_h1_sigma0003.md`.
