# Protocollo H0 V2 — adattamento a `primary_grf_split_v1` con teacher prescritto

Data: 2026-08-06

## Motivazione e provenienza

Il ramo V1 resta immutabile e terminale con
`ERROR_PRIMARY_SPLIT_TEACHER`. I tre rollout H0 controfattuali sui trial
02/04/08 si sono arrestati a 59/66/115 step per penetrazione della GRF primaria
oltre 25 mm, senza completare cicli. Un fit perfetto sulle azioni di quel
teacher riprodurrebbe quindi una behavior già instabile e non può costituire
una soluzione.

L'autorizzazione utente del 2026-08-06 apre un nuovo ramo no-clobber e chiede di
avanzare fino al gate più vicino a `CORRIDOR_TRAINING_READY` senza richiedere
decisioni intermedie. Il ramo si chiama `H0_primary_split_v2`.

## Obiettivo e limite

Output massimo di questo protocollo:

```text
H0_PRIMARY_GRF_SPLIT_V2_BASELINE_READY
```

Il protocollo non apre trial protetti, non promuove V25, non avvia PPO e non
abilita morphology reward positivo. Un suo PASS autorizza soltanto un nuovo
preflight development A/B/C con V25/V20.

## Invarianti

- H0 sorgente resta immutabile.
- Layout actor/full `35/84`, ordine e dtype `float32` restano invariati.
- La GRF primaria sinistra resta l'unica sorgente online del carico continuo e
  del contatto fisico dell'actor.
- La GRF destra prescritta resta applicata nel supporto ibrido.
- Gli eventi restano `legacy_events` e V25/V20 restano disabilitati in questo
  ramo.
- Detector analogico e binario restano force-free e non possono guidare
  azioni, label o dinamica.
- `morphology_weight=0` e zero update PPO/critic.
- Nessuna modifica a GRF primaria, geometrie, plugin C++ o semantica SEA.
- Trial 05/06 e reserve 03/07 restano chiusi.
- Ogni artefatto è strict, finito, atomico e no-clobber.

## Teacher e semantica delle label

Il teacher offline è la cinematica protesica prescritta dello stesso trial,
campionata al successivo boundary policy e codificata nell'azione assoluta
normalizzata attraverso gli stessi bounds runtime. L'azione attraversa gli
stessi slew limiter, reference governor, SEA e plant usati dal candidato.

Il teacher:

- guida soltanto i rollout di raccolta e le reference di qualificazione;
- non è una feature dell'actor;
- non è disponibile nel runtime candidato;
- viene calcolato dopo la scelta dell'azione nei rollout autonomi, soltanto per
  diagnostica controfattuale.

H0 è usato esclusivamente come inizializzazione e weight anchor. Non guida il
plant e non fornisce le label. Questo evita di clonare la failure V1.

## Dati e split

Development già aperto, plateau 04:

| Ruolo | Trial | Velocità | Tempo assoluto | Offset | Seed |
|---|---:|---:|---:|---:|---:|
| train | 02 | 0,95 m/s | 119,578 s | 107,880 s | 123 |
| train | 04 | 1,05 m/s | 122,189 s | 107,550 s | 124 |
| holdout offline | 08 | 1,25 m/s | 120,390 s | 106,878 s | 125 |

Ogni reference è deterministica e deve produrre 500 coppie
`target_actor_observation[35] -> prescribed_action[2]`. Lo split è per trial:
02/04 train (1000 righe), 08 validation (500 righe), senza leakage.

Gate teacher per ogni trial:

- 500/500 step, time limit e almeno due cicli;
- penetrazione finita `<0,025 m`;
- zero safety stop, timeout, clipping, fallback, hard-invalid e non-finiti;
- layout `35/84`, routing e contratto eventi esatti;
- tutte le metriche reserve/residual/SEA finite;
- zero PPO, critic update e accessi protetti.

Un FAIL chiude V2 senza cambiare finestra, soglia o teacher.

## Fit actor-only one-shot

Un solo candidato, senza sweep:

```text
optimizer                 Adam
learning_rate             5e-5
epochs_max                400
batch_size                128
patience                  60
anchor_weight             1e-2
clip_weight               1.0
logstd_weight             0.0
seed                       123
trainable_scope           full mean actor
logstd                     frozen bit-exact
critic/non-actor           unchanged
```

Gate offline sul trial 08:

- RMSE candidato-vs-prescritto `<=0,03`;
- max abs error `<=0,15`;
- riduzione RMSE di almeno 50% rispetto a H0;
- zero output non finiti o fuori `[-1,1]` sull'intero corpus;
- actor digest diverso da H0;
- logstd, critic/non-actor e save/reload bit-exact dove applicabile;
- manifest esplicito e ordinato delle 35 feature.

## Qualificazione closed-loop

Finestre non usate dal fit:

| Trial | Tempo assoluto | Offset | Seed |
|---|---:|---:|---:|
| 02 | 129,578 s | 117,880 s | 126 |
| 04 | 132,189 s | 117,550 s | 127 |
| 08 | 130,390 s | 116,878 s | 128 |

Per ogni trial si eseguono reference prescritta e candidato autonomo in due
modalità: deterministica e stocastica con sigma H0 `0,005` e tape congelato.
Prima devono passare tutte le sei reference; poi possono partire i sei
candidati.

Ogni rollout deve rispettare i gate fisici teacher. In più il candidato deve
avere, sui propri stati visitati:

- RMSE mean-vs-prescritto `<=0,03`;
- max abs `<=0,15`;
- zero dipendenza dell'azione servita dal teacher;
- reserve/residual e metriche SEA non superiori alla reference condition-matched
  oltre `max(15%, 25 Nm)` per reserve, `max(15%, 1e-6 Nm)` per residual e
  `max(15%, 1e-6)` per le metriche SEA nelle rispettive unità;
- saturazioni e conteggi di errore non superiori alla reference.

Un FAIL è terminale e non genera DAgger, retuning o un secondo candidato.

## Port zero-update

Solo dopo il PASS closed-loop:

1. scrivere e congelare manifest actor 35-feature e config target;
2. costruire un trainer nuovo con critic e optimizer freschi;
3. trapiantare soltanto l'actor senza chiamare `train()`;
4. verificare actor bit-exact su learner, EnvRunner ed export;
5. verificare critic invariato rispetto alla sua inizializzazione e optimizer
   con stato vuoto;
6. salvare e ricaricare il checkpoint completo a iteration/update/sample zero;
7. verificare nuovamente actor, critic e optimizer.

Il PASS produce `H0_PRIMARY_GRF_SPLIT_V2_BASELINE_READY`. Il warm-up critic e
qualsiasi PPO restano chiusi.

## Arresti e passo successivo

Esiti terminali:

- `ERROR_H0_PRIMARY_SPLIT_V2_TEACHER`;
- `FAIL_H0_PRIMARY_SPLIT_V2_OFFLINE`;
- `FAIL_H0_PRIMARY_SPLIT_V2_CLOSED_LOOP`;
- `ERROR_H0_PRIMARY_SPLIT_V2_ZERO_PORT`;
- `H0_PRIMARY_GRF_SPLIT_V2_BASELINE_READY`.

Dopo il PASS si congela una nuova matrice A/B/C development per il checkpoint
V2. Il ramo H0/V25 del 2026-08-05 non viene riaperto. L'apertura one-shot del
trial protetto 05 resta una decisione utente separata.
