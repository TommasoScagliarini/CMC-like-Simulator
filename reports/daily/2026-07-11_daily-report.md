# Daily Report - 2026-07-11

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `reports/user/2026-07-11_rollout_diagnostico_warm_start_iniziale.md`
- `reports/user/2026-07-11_riadattamento_imitativo_target_domain.md`

## Sintesi

La giornata ha verificato il comportamento reale del warm start prima di PPO,
individuato un errore nella semantica del target slew limiter e costruito un
actor target-domain mediante behavior cloning e DAgger.

Il porting tensoriale era corretto, ma non garantiva compatibilita'
comportamentale con il contratto ex-novo. Il primo rollout terminava prima del
TO; dopo la correzione del limiter il teacher ha completato l'episodio, mentre
il miglior actor adattato ha raggiunto un ciclo valido ma non `500/500` step.
Il transplant PPO a zero iterazioni e' passato, mentre il preflight stocastico
ha rivelato una varianza ereditata incompatibile con l'action space. H1 non e'
stata avviata il giorno 11.

## 1. Rollout diagnostico iniziale

Il modulo `rl_module_initial_warm_start`, valutato deterministicamente prima di
qualsiasi gradiente PPO con reward e soglie `15/25 mm` congelate, ha prodotto:

```text
step                              35 / 500
return                            -2.148
HS / TO / cicli                   1 / 0 / 0
penetrazione massima              26.106 mm
clipping action                   6 / 35 step
fine                              grf_penetration
```

Due controfattuali hanno escluso spiegazioni semplici:

- portare il guard a `30 mm` ha aggiunto solo quattro step senza raggiungere il
  TO;
- rimuovere lo slew limiter ha consentito un ciclo a 195 step, ma con ciclo
  troppo rapido, clipping, reserve fino a circa `851 Nm` e accelerazione motore
  del ginocchio oltre `6300 rad/s^2`.

Il problema non era quindi il solo guard. Esisteva un domain shift tra actor
imitativo e contratto target con clock disabilitato, FSM nuova, limiter e
reference governor.

## 2. Causa radice nel target slew limiter

Anche il teacher prescribed falliva prima del TO. I probe con lookahead
temporali di ginocchio e caviglia arrivavano al massimo a circa 60 step, quindi
il ritardo del governor non spiegava il problema.

Il limiter documentato come target-to-target usava invece come anchor la
reference gia' filtrata. Il limite nominale `2.5/2.0 rad/s` diventava in regime
circa `0.31/0.25 rad/s`, impedendo al target di avanzare cumulativamente.

La correzione in `Trajectory Generator/osim_trj_cmc_like.py` ha separato:

- `target_anchor`: ultimo endpoint policy accettato, usato per il rate limit;
- `continuity_anchor`: reference filtrata corrente, usata per la continuita'
  del segmento.

Reward, soglie, governor, action mode, plugin SEA e stato iniziale non sono
stati modificati. Un regression test verifica che il limiter avanzi rispetto
all'ultimo target accettato.

Con il limiter corretto il teacher ha completato il contratto target:

```text
step                              500 / 500
HS / TO / cicli                   3 / 3 / 2
return                            +64.032
penetrazione massima              22.944 mm
gate                              PASS
```

Questo ha dimostrato che la pipeline e la soglia da 25 mm ammettono una
traiettoria completa.

## 3. Adattamento imitativo dell'actor

Un primo behavior cloning actor-only su 500 coppie osservazione target e
azione prescribed ha ottenuto un buon fit offline, ma in closed loop si e'
fermato a 68 step. La causa era il covariate shift: piccoli errori d'azione
modificavano reference, SEA, contatto e osservazione successiva, portando la
rete fuori dalla singola traiettoria usata per il training.

E' stato quindi applicato DAgger, raccogliendo gli stati visitati dall'actor e
facendoli etichettare dal teacher. I risultati principali sono stati:

| Candidato | Step | Return | HS/TO/cicli | Clip | Decisione |
| --- | ---: | ---: | ---: | ---: | --- |
| source ported | 396 | -7.213 | 2/2/1 | 3.66% | non promosso |
| behavior cloning | 68 | -3.385 | 1/0/0 | 0% | respinto |
| DAgger round 1 | 45 | -2.165 | 1/0/0 | 0% | respinto |
| DAgger round 2 | 356 | +29.886 | 2/2/1 | 0% | selezionato |
| DAgger round 3 | 221 | -56.848 | 1/0/0 | 0% | respinto |

Il round 2 ha superato il gate minimo `valid_cycle_count >= 1`, ma non il full
episode: e' terminato per penetrazione a `26.512 mm` e ha mostrato un picco
reserve di circa `978 Nm`. Il round 3 ha confermato che piu' DAgger non produce
un miglioramento monotono.

Checkpoint selezionato:

```text
Trajectory Generator/runs/training/
target_domain_dagger_2026-07-11_r2/rl_module_target_adapted/
actor digest: 853940898f079d560756ce22aaa1c908239b4dfa2da82ce690a2fd08d3558c27
```

## 4. Gate pre-H1

Il round 2 e' stato congelato con manifest esplicito a 39 feature e trapiantato
nel trainer PPO completo con zero iterazioni.

```text
digest learner / EnvRunner / export       identico al source
critic target                              invariato, 6 tensori exact
optimizer imitativo importato              no
output su osservazioni sintetiche/reali    equivalente
gate porting                               PASS
```

Il preflight `forward_exploration` ha pero' fallito su tutti i seed:

```text
seed 123/124/125                 48 / 50 / 77 step
TO / cicli                       0 / 0
clipping mediano                 32%
terminazioni per penetrazione    3 / 3
```

La deviazione standard ereditata era state-dependent, con mediana circa
`0.75` e massimo oltre `2.45` nell'action space `[-1, 1]`. La media actor
selezionata non doveva essere modificata; il passo successivo era configurare
solo la testa `log-std` e ripetere il gate.

Stato finale del giorno:

```text
teacher target-domain             PASS
actor DAgger round 2              PASS minimo, non full-episode
porting PPO zero-iteration        PASS
preflight stocastico              FAIL
ready_for_h1                      false
decisione                         STOP_BEFORE_H1
```

## File modificati

- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/baseline_MLP/target_domain_imitation.py`
- `Trajectory Generator/baseline_MLP/target_domain_dagger.py`
- `Trajectory Generator/baseline_MLP/freeze_actor_checkpoint.py`
- `Trajectory Generator/baseline_MLP/warm_start.py`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `validation/test_target_domain_imitation.py`
- `validation/test_target_slew_limiter.py`
- `validation/test_warm_start.py`
- `validation/test_rollout_eval.py`
- `validation/validate_warm_start_port.py`
- `validation/summarize_warm_start_preflight.py`
- `validation/test_warm_start_preflight.py`

## Test e verifiche

- `py_compile`: PASS;
- unit test: `33/33` PASS;
- config smoke: PASS;
- regression target-to-target del limiter: PASS;
- teacher full episode: PASS;
- save/reload actor: bit-exact;
- porting a learner ed EnvRunner: PASS;
- `git diff --check`: PASS;
- rollout DAgger round 2: un ciclo valido, full episode FAIL;
- preflight stocastico PPO: FAIL controllato, nessuna H1 eseguita.

## TODO aperti e propagati

- [ ] Reinizializzare esclusivamente `log-std`, preservando bit-exact la media
      DAgger round 2, e ripetere il gate stocastico su tre seed.
- [ ] Eseguire una sola H1 warm-start e H2 soltanto dopo il superamento del
      gate; mantenere il round 2 come rollback immutabile.
- [ ] Richiedere a H2 almeno il full episode o un miglioramento misurabile
      rispetto a 356 step, oltre a cicli, penetrazione, reserve e clipping.
- [ ] Confrontare il risultato warm-start con la fresh policy sul failure mode
      e non sbloccare training lunghi senza un TO/ciclo robusto.
- [ ] Spiegare il TO precoce rifiutato nella seconda stance dell'oracolo
      multi-ciclo, TODO ereditato dal 10 luglio.
- [ ] Tenere una memoria esplicita in coda finche' i test di osservabilita' non
      mostrino un limite sequenziale reale; non introdurla come scorciatoia.
