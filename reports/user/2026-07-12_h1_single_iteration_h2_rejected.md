# H1 singola e H2: checkpoint respinto

Data: 2026-07-12

## Obiettivo

Eseguire il protocollo concordato senza avviare un training lungo:

1. una sola iterazione PPO H1 dal warm start DAgger r2;
2. `sigma=0.003` e `freeze_logstd=true`;
3. audit immediato dei pesi;
4. H2 deterministico e tre seed stocastici;
5. verifica aggiuntiva a `sigma=0.005`, livello tollerato dal teacher;
6. stop obbligatorio in assenza di miglioramento.

Reward, soglie, FSM, limiter e actor iniziale non sono stati modificati.

## Contratto H1

E' stato riutilizzato direttamente lo YAML risolto del trainer zero-iteration
gia' validato. L'unica modifica al contratto e' stata:

```text
iterations                       0 -> 1
```

Configurazione principale:

```text
warm-start source                DAgger r2 sigma=0.003
actor iniziale digest            7fa7fbd15db67dd29b9cd428528b68e14648e934f70c165d6d6be901f025383c
freeze_logstd                    true
sigma knee/ankle                 0.003 / 0.003
train batch                      4096
minibatch                        512
epochs                           10
learning rate                    1e-4
EnvRunner                        13
start                            fisso
soglie penetrazione              15 / 25 mm
```

Il primo lancio in sandbox e' fallito in `ray.init()` per il divieto macOS di
leggere la process list. Non era ancora avvenuto il transplant e non e' stato
eseguito alcun update. Il run valido e' stato quindi rilanciato fuori sandbox
con path assoluti.

## Esito H1

H1 e' terminata correttamente in 489 s:

```text
iterazioni riuscite              1
step campionati                  4096
episode return mean              -16.9721
episode length mean              210.778
terminazioni per penetrazione    8
timeout di stance                1
policy loss                      0.010120
value loss                       1.084395
value explained variance         0.112041
mean KL loss                     0.391552
```

Il batch era quindi dominato da episodi che fallivano intorno al primo TO o
prima del ciclo completo. Non ci sono stati timeout di training, restart Ray o
iterazioni saltate.

## Audit dei pesi

Digest:

```text
actor iniziale                   7fa7fbd15db67dd29b9cd428528b68e14648e934f70c165d6d6be901f025383c
actor post-H1                    13d7f6786b0dfea593861b30c92e3895d63e1eb93293f77e82745e03e4e069a3
rl_module_best = rl_module_last  si
```

La testa esplorativa e' rimasta bit-exact:

```text
log-std weight                   invariato esattamente
log-std bias                     invariato esattamente
sigma post-H1                    0.003 / 0.003
```

La media e' cambiata, come previsto. Sui 356 stati nominali del rollout r2:

```text
massimo spostamento action mean  0.007263
RMSE complessivo                 0.002119
RMSE knee / ankle                0.001461 / 0.002617
p95 abs knee / ankle             0.002722 / 0.005186
massima variazione parametro     0.002551
```

La variazione e' piccola in valore assoluto, ma non rispetto a
`sigma=0.003`: il massimo spostamento vale circa `2.42 sigma`. Questo rende
coerente il KL elevato osservato, nonostante pesi apparentemente poco diversi.

## H2 deterministico

Il primo tentativo H2 parallelo e' stato invalidato: quattro OpenSim simultanei
hanno saturato la macchina e tutti i watchdog si sono fermati intorno agli step
112-114. I tentativi successivi sono stati seriali.

Due tentativi deterministici seriali hanno incontrato stalli OpenSim
intermittenti. Un controllo A/B di 10 step ha poi completato sia pre-H1 sia
post-H1, escludendo una divergenza immediata del runtime. Il terzo full rollout
seriale ha prodotto un risultato completo e viene usato come H2 valido:

```text
                         pre-H1             post-H1
step                     356                221
return                   +29.886            -59.689
HS / TO / cicli          2 / 2 / 1          1 / 0 / 0
clipping                 0%                 0%
max penetrazione         26.512 mm          22.536 mm
fine                     penetrazione       stance timeout
```

H1 ha quindi distrutto il TO e il ciclo anche in modalita' deterministica.

## H2 stocastico a sigma 0.003

```text
seed   pre step/cicli/return        post step/cicli/return       fine post
123    361 / 1 / +31.092            214 / 0 / -18.654           penetrazione
124    360 / 1 / +31.496            221 / 0 / -57.928           stance timeout
125    210 / 0 / -17.892            213 / 0 / -18.967           penetrazione
```

Aggregato:

```text
mediana step pre -> post           360 -> 214
seed con TO pre -> post            3/3 -> 2/3
seed con ciclo pre -> post         2/3 -> 0/3
clipping post                      0%
```

La regressione non e' limitata al replay deterministico.

## Probe post-H1 a sigma 0.005

E' stata creata una copia diagnostica modificando esclusivamente le due bias
`log_std`. La media post-H1 e' stata verificata bit-exact su 356 stati e non e'
stato eseguito altro PPO.

```text
seed   step   return     HS/TO/cicli   fine
123     207   -15.512       1/1/0       penetrazione
124     221   -57.851       1/0/0       stance timeout
125     214   -18.916       1/1/0       penetrazione
```

```text
mediana step                       214
seed con TO                        2/3
seed con ciclo                     0/3
```

Non e' stato osservato alcun miglioramento verso `sigma=0.005`.

## Interpretazione

Il fallimento non e' causato da una modifica accidentale di sigma: la testa
`log_std` e' rimasta esattamente congelata. E' la media ad essersi spostata in
una regione closed-loop peggiore.

Tre fattori spiegano il risultato:

1. con `sigma=0.003`, spostamenti dell'ordine di pochi millesimi rappresentano
   gia' una grande distanza tra distribuzioni;
2. dieci epoch sul primo batch hanno prodotto `mean KL=0.392`, quindi il primo
   update non e' stato sufficientemente trust-region rispetto al prior;
3. il critic target era intenzionalmente fresco e il batch iniziale era quasi
   interamente composto da episodi falliti, rendendo il primo segnale di
   vantaggio poco affidabile per modificare subito l'actor.

Questi dati non dimostrano che la reward sia errata. Dimostrano che l'attuale
procedura di primo update PPO non protegge una policy warm-start estremamente
sensibile.

## Decisione

```text
stato checkpoint H1                REJECT_H1
promuovere rl_module_best          no
eseguire iterazione 2              no
proseguire training lungo          no
checkpoint da conservare           warm start pre-H1 sigma=0.003
```

Il prossimo protocollo non deve ripetere H1 invariata. La priorita' e': 

1. warm-up del critic mantenendo l'actor completamente congelato;
2. primo update actor con budget KL esplicito e molto piu' conservativo;
3. ridurre epoch e learning rate dell'actor oppure usare optimizer separati;
4. introdurre rollback automatico se H2 deterministico regredisce;
5. mantenere il checkpoint pre-H1 come baseline immutabile.

Una possibile H1-bis va considerata solo dopo avere implementato questi vincoli.

## Artefatti

```text
training H1:
validation/warm_start_h1_runs/2026-07-12_sigma0003_single_iteration/

gate machine-readable:
validation/warm_start_h1_runs/2026-07-12_sigma0003_single_iteration/h1_h2_gate.json

H2 sigma 0.003:
validation/warm_start_h2_runs/2026-07-12_sigma0003_after_h1/

H2 sigma 0.005:
validation/warm_start_h2_runs/2026-07-12_sigma0005_after_h1/
```

## TODO

- [x] Eseguire una sola H1 con `sigma=0.003` e `freeze_logstd=true`.
- [x] Verificare digest, media e log-std prima/dopo.
- [x] Eseguire H2 deterministico e tre seed a `sigma=0.003`.
- [x] Verificare il checkpoint post-H1 a `sigma=0.005`.
- [x] Fermarsi senza iterazione 2 dopo il gate fallito.
- [ ] Progettare e validare critic warm-up e update actor con vincolo KL.
