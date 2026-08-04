# Corridor morfologico retrospettivo sperimentale

Data: 2026-07-20

## Sintesi esecutiva

È stata implementata una terza modalità del Morphology Corridor, separata sia
dal metodo storico sia dal metodo causale attualmente configurato:

```text
event_anchored_completed_segment_experimental
```

La nuova modalità esegue esattamente il comportamento richiesto sulla forma
temporale del corridor:

- la traiettoria `served` non viene modificata, riscampionata o filtrata;
- la FSM fornisce gli eventi protesici HS e TO accettati;
- la porzione di corridor in stance viene stirata o compressa tra HS e TO;
- la porzione di corridor in swing viene stirata o compressa tra TO e il
  successivo HS;
- corridor e served usano la stessa convenzione di segno anatomica/OpenSim;
- la coordinata ankle mantiene il segno raw del modello, positivo in
  dorsiflessione;
- l'esatto endpoint futuro non viene inventato: i campioni sono valutati solo
  quando il segmento è completo.

Il metodo resta intenzionalmente **non definitivo**:

- `training_exnovo_cfg.yaml` continua a usare il metodo corrente
  `event_anchored`;
- il nuovo metodo vive in una configurazione separata sotto
  `experimental_configs/`;
- `morphology_weight = 0`;
- la terminazione hard è disattivata;
- un guard impedisce di attivare per errore peso o terminazione nel training;
- il metodo storico e quello corrente non sono stati rimossi.

Verdetto:

> La nuova geometria HS-TO-HS è validata come alternativa sperimentale shadow.
> Non è ancora promossa a reward PPO attiva e non sostituisce il metodo
> `event_anchored` attualmente selezionato.

## Problema affrontato

Il mapping causale `event_anchored` implementato in precedenza non conosce il
prossimo evento futuro. Durante il segmento deve quindi usare una durata
nominale o una durata stimata dai cicli passati:

```text
stance: phase = alpha * clip((t - HS) / T_stance_stimata, 0, 1)
swing:  phase = alpha + (1-alpha) * clip((t - TO) / T_swing_stimata, 0, 1)
```

Se l'evento reale arriva dopo la durata stimata, la fase rimane saturata
all'estremo del segmento. Nel grafico questo appare come un plateau o un
gradino. Non è un gradino introdotto nella served: è il corridor causale che
attende il prossimo evento senza poter conoscere in anticipo quando avverrà.

La richiesta era diversa: lasciare completamente invariata la served e
stirare/restringere la forma del corridor usando gli eventi reali HS-TO-HS
accettati dalla FSM.

## Soluzione implementata

### Mapping esatto per segmento completato

Il profilo event-warped usa l'ancora canonica:

```text
alpha = 0.6223299989
```

Per un segmento di stance completo:

```text
HS_i <= t < TO_i

phase(t) = alpha * (t - HS_i) / (TO_i - HS_i)
```

Per il successivo segmento di swing completo:

```text
TO_i <= t < HS_(i+1)

phase(t) = alpha
         + (1 - alpha) * (t - TO_i) / (HS_(i+1) - TO_i)
```

La stessa forma canonica viene quindi compressa nei segmenti brevi e stirata
nei segmenti lunghi. Non viene trasformata la cinematica generata dalla rete:
il remapping riguarda esclusivamente il punto del profilo morfologico con cui
ogni campione served viene confrontato.

### Ownership delle frontiere

È stato adottato un contratto half-open non ambiguo:

```text
stance = [HS, TO)
swing  = [TO, next_HS)
```

Il campione esattamente sull'evento appartiene al segmento appena aperto.
Questo elimina doppie assegnazioni e perdite alle frontiere.

### Eventi retrodatati e buffer

Il detector può confermare un evento alcuni step dopo il suo timestamp fisico.
La FSM ora pubblica un journal diagnostico degli eventi accettati contenente:

- evento e timestamp fisico;
- stato FSM precedente e successivo;
- tipo e inizio del segmento chiuso;
- validità geometrica del segmento;
- tipo del segmento aperto;
- validità del ciclo e motivo di eventuale rejection.

Il journal non entra nell'osservazione dell'actor. Il ledger mantiene un buffer
dei campioni served e, quando arriva la conferma, lo ripartisce usando il
timestamp fisico dell'evento, non il tempo di conferma.

Nel rollout reale le latenze di conferma sono comprese tra 6 e 60 ms. Il
replay ha verificato che `time - elapsed` restituisce lo stesso identico anchor
in ogni riga di ciascun tratto FSM.

### Segno anatomico

Non viene applicata alcuna inversione al runtime:

```text
served knee  transform = +1
corridor knee transform = +1
served ankle transform = +1
corridor ankle transform = +1
```

Il knee può essere invertito soltanto a scopo di visualizzazione
flexion-positive, purché la stessa trasformazione venga applicata a served e
corridor. Il plot prodotto in questa validazione usa invece il segno raw per
entrambi. Per l'ankle il titolo dichiara esplicitamente:

```text
positive = dorsiflexion
```

### Semantica corrente della loss

Per ogni coordinata il corridor interno rimane:

```text
lower = mean - K * std - margin
upper = mean + K * std + margin
```

Dentro i bordi la penalità morfologica è costante e nulla. Fuori dai bordi
cresce quadraticamente con l'escursione normalizzata:

```text
distance = distanza dal bordo più vicino
width    = upper - lower
loss     = min(25, (distance / width)^2)
```

Questo conserva la logica già esistente: libertà piena dentro il corridor e
pressione crescente fuori dal corridor.

La loss esatta dei campioni passati diventa nota soltanto alla chiusura del
segmento. Il wrapper può quindi loggarne la somma sullo step di chiusura, ma
non può retroattivamente riscrivere le reward già consegnate a PPO.

### Terminazione hard: stato volutamente provvisorio

È presente soltanto un hook sperimentale disattivato, basato su limiti globali
phase-independent delle served. Viene verificato alla chiusura del segmento.

Non è ancora il bordo esterno definitivo del corridor a tre zone e non è una
terminazione immediata. Un segmento incompleto, per definizione, non possiede
ancora la phase esatta rispetto all'endpoint futuro.

La configurazione fornita mantiene:

```json
"morphology_hard_termination_enabled": 0.0,
"morphology_experimental_allow_effects": 0.0
```

Questa parte deve essere progettata e validata separatamente prima di una
promozione.

## Isolamento sperimentale e possibilità di scelta

Restano disponibili tre contratti distinti:

| Modalità | Stato | Caratteristica |
|---|---|---|
| `legacy_cycle_fraction` | disponibile | comportamento storico |
| `event_anchored` | configurazione corrente | causale, utilizzabile per-step, usa timing passato/nominale |
| `event_anchored_completed_segment_experimental` | nuova, shadow | geometria esatta su segmenti completati |

La configurazione operativa non è stata cambiata:

```yaml
morphology_phase_mode: event_anchored
morphology_weight: 0.0
```

La nuova variante si seleziona soltanto tramite:

```text
Trajectory Generator/baseline_MLP/experimental_configs/
  morphology_completed_segment_shadow.json
```

Il costruttore dell'ambiente fallisce immediatamente se la modalità
sperimentale richiede un peso positivo o una terminazione hard senza un opt-in
esplicito riservato ai test sintetici.

## Validazione sintetica e regressioni

### Ledger e FSM

La nuova suite copre:

- invariance a stretch e compressione;
- eventi HS/TO confermati in ritardo;
- ripartizione del buffer al timestamp fisico;
- ownership half-open senza duplicazioni o perdite;
- mismatch del timestamp iniziale, incluso `NaN`;
- rejection anti-fake-cycle separata dalla validità geometrica;
- timeout FSM;
- reset e fine episodio;
- fine episodio con segmento attivo ma nessun campione finito;
- timestamp duplicati e non monotoni;
- overflow del buffer fail-closed;
- preservazione di un HS a `t = 0`;
- peso zero bit-exact;
- settlement della loss soltanto alla chiusura;
- hard hook inclusivo sui bound e ritardato;
- precedenza di `numerical_failure` e `step_wall_timeout` sulla terminazione
  morfologica.

Esito:

```text
19/19 PASS
```

### Replay fail-closed

Il replay da trace rifiuta:

- contatori FSM che diminuiscono o saltano;
- incremento simultaneo HS e TO;
- ciclo valido senza HS di chiusura;
- ordine HS-TO-HS non valido;
- elapsed clock incoerente con l'anchor recuperato;
- periodo o stance fraction incoerenti con la diagnostica registrata.

Esito:

```text
4/4 PASS
```

### Regressioni

```text
event-anchored corrente       11/11 PASS
reward esistente              32/32 PASS
contratto rollout              5/5 PASS
training config smoke          tutti PASS
py_compile                     PASS
Ruff                           PASS
Black --check                  PASS
git diff --check               PASS
```

Il validatore indipendente include anche un self-test positivo e un probe
fatale che deve fallire su overflow, timestamp non monotono o start mismatch.

## Validazione sul rollout reale del checkpoint best

### Sorgente

È stata usata la trace deterministica reale da 500 step già completata con il
checkpoint best, stesso start nominale e seed 123. Il replay:

1. non esegue nuovamente la policy;
2. non modifica alcun campo preesistente della trace;
3. ricostruisce gli eventi accettati dai contatori FSM e dagli elapsed clock;
4. passa le served originali nel ledger di produzione;
5. consegna journal e payload al validatore indipendente.

### Eventi recuperati

| Evento | Timestamp fisico [s] | Riga conferma | Latenza [s] |
|---|---:|---:|---:|
| HS1 | 13.946870984 | 5 | 0.060 |
| TO1 | 15.217870984 | 127 | 0.009 |
| HS2 | 15.488870984 | 159 | 0.058 |
| TO2 | 16.590870984 | 264 | 0.006 |
| HS3 | 16.879870984 | 298 | 0.057 |
| TO3 | 18.258870984 | 431 | 0.008 |
| HS4 | 18.508870984 | 461 | 0.058 |

Sono stati ricostruiti tre cicli completi, con durate:

| Ciclo | Stance [s] | Swing [s] | Periodo [s] |
|---|---:|---:|---:|
| 1 | 1.271 | 0.271 | 1.542 |
| 2 | 1.102 | 0.289 | 1.391 |
| 3 | 1.379 | 0.250 | 1.629 |

### Risultati del ledger

| Metrica | Risultato |
|---|---:|
| segmenti stance completati | 3 |
| segmenti swing completati | 3 |
| campioni settled | 456 |
| campioni duplicati | 0 |
| campioni persi nei segmenti completi | 0 |
| campioni scartati | 44 |
| motivo dello scarto | stance finale incompleta a fine episodio |
| copertura grezza | 91.2% |
| copertura corretta per bootstrap/coda | 100% |
| overflow | 0 |
| timestamp non monotoni | 0 |
| mismatch dell'inizio segmento | 0 |

I 44 campioni non sono un errore del ledger. Dopo HS4 inizia una nuova stance,
ma il rollout termina prima del TO successivo; manca quindi l'endpoint futuro
necessario per definire la compressione esatta.

### Fase, corridor e sign

Il validatore indipendente ha riportato:

```text
overall gate                         PASS
phase plateau count                 0
phase non-monotonic increment count 0
phase interval violations           0
minimum positive phase increment    0.0045129079
phase max reconstruction error      0
corridor-bound max error             0 rad
loss max error                       0
served-reference max error           0 rad
time max error                       0 s
raw sign identity                    PASS
```

Questo risponde ai due problemi osservati nel grafico precedente:

- i plateau/gradini della fase causale non sono presenti nei segmenti
  completati del nuovo mapping;
- ankle served e ankle corridor hanno la stessa convenzione di segno raw,
  positivo in dorsiflessione.

### Morfologia della served corrente

Sui 456 campioni con phase esatta:

| Metrica | Knee | Ankle |
|---|---:|---:|
| frazione dentro il corridor | 86.184% | 76.535% |
| loss cumulativa per giunto | 50.0510 | 11.9275 |

La loss morfologica combinata è:

```text
somma = 30.9892439915
media = 0.0679588684
```

Questi valori sono diagnostici: il peso è zero e quindi non modificano il
return.

### Invarianza della trace

Il confronto tra la trace sorgente e la trace arricchita ha restituito:

| Canale | RMSE | Massimo assoluto |
|---|---:|---:|
| raw policy action | 0 | 0 |
| applied policy action | 0 | 0 |
| cinematica protesica | 0 | 0 |
| served reference | 0 | 0 |

Sono rimasti identici anche:

```text
step                         500
return                       52.42693952981385
cicli validi                 3
penetrazione GRF massima     0.02421743110349864 m
end reason                   episode_time_limit
```

Questa verifica dimostra che il replay diagnostico non altera i dati reali.
Non sostituisce però un nuovo rollout live della modalità sperimentale.

## Tentativi di rollout live della nuova modalità

Sono stati avviati due rollout freschi del checkpoint best con configurazione
shadow:

1. run da 500 step: watchdog in `inference simulator step 176`;
2. run limitata a 175 step: watchdog in `inference simulator step 22`.

In entrambi i casi il watchdog ha osservato oltre 300 s senza heartbeat dentro
la chiamata di integrazione del simulatore e ha terminato il worker. Non è
stata prodotta una trace parziale utilizzabile e nessuno dei due tentativi è
stato contato come PASS.

Il diverso punto di stallo, insieme alla dimensione trascurabile del ledger,
è compatibile con un problema di esecuzione OpenSim non deterministico, ma non
costituisce una prova sufficiente per escludere qualsiasi interazione. Per
questo il rollout live completo resta un gate aperto prima della promozione.

I due watchdog sono conservati come evidenza nelle cartelle:

```text
Trajectory Generator/runs/rollout/validation/experimental_morphology_runs/
  2026-07-20_completed_segment_shadow_best_failed_watchdog_step176/
  2026-07-20_completed_segment_shadow_best_failed_watchdog_step22/
```

## Perché il metodo non è ancora pronto per un training PPO positivo

PPO usa attualmente frammenti fixed-step, `gamma` e GAE. La loss esatta dei
campioni di una stance o di uno swing diventa disponibile soltanto quando
l'evento futuro chiude il segmento.

Sottrarre tutta la somma sul solo step TO/HS non è equivalente a distribuire le
loss sui rispettivi step originali: cambia il discounting e quindi gli
advantage. Il runtime espone questa somma per diagnostica, ma il guard impedisce
di usarla accidentalmente nel training.

Per rendere la variante protagonista nel training serve un protocollo
complete-episode o complete-segment che:

1. conservi reward e transizioni fino alla chiusura del segmento;
2. ricostruisca la phase esatta;
3. riscriva la reward di ciascun campione originale;
4. calcoli GAE soltanto dopo la riscrittura;
5. gestisca in modo esplicito segmenti incompleti, timeout e bootstrap.

## File modificati e aggiunti

### Runtime sotto `Trajectory Generator/`

```text
Trajectory Generator/prosthetic_phase_fsm.py
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/experimental_morphology_corridor.py
Trajectory Generator/baseline_MLP/experimental_configs/
  README.md
  morphology_completed_segment_shadow.json
```

Non sono stati modificati il plugin C++, la semantica del comando SEA, le
azioni, le served reference o lo schema di input dell'actor.

### Script di test e validazione

Tutti gli script introdotti sono dentro `validation/`:

```text
validation/test_experimental_retrospective_morphology.py
validation/validate_experimental_retrospective_morphology.py
validation/replay_completed_segment_morphology.py
validation/test_replay_completed_segment_morphology.py
```

Sono stati aggiornati i test preesistenti necessari al contratto rollout e
alla reward.

## Artefatti

### Replay reale

```text
validation/experimental_morphology_runs/
  2026-07-20_completed_segment_real_trace_replay/
    rollout_policy_trace.json
    rollout_summary.json
    completed_segment_replay_summary.json
```

### Validazione indipendente

```text
validation/experimental_morphology_runs/
  2026-07-20_completed_segment_real_trace_validation/
    experimental_retrospective_morphology_summary.json
    experimental_retrospective_morphology_time_overlay.png
```

Confronto di invarianza:

```text
validation/experimental_morphology_runs/
  2026-07-20_completed_segment_real_trace_invariance.json
```

Plot utente:

```text
plot/07_20_2026_morphology_completed_segment_experimental/
  01_morphology_corridor_exact_hs_to_hs.png
```

Nel plot la linea nera della served è continua sull'intera trace. Il corridor
viene mostrato soltanto sui segmenti con entrambi gli eventi noti; la coda
finale non valutabile è tratteggiata, non interpolata artificialmente.

## Comandi di riproduzione

Replay della trace reale:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python \
  validation/replay_completed_segment_morphology.py \
  "Trajectory Generator/runs/rollout/validation/event_anchored_morphology_runs/2026-07-20_weight0_live_rollout_final/rollout_policy_trace.json" \
  --summary "Trajectory Generator/runs/rollout/validation/event_anchored_morphology_runs/2026-07-20_weight0_live_rollout_final/rollout_summary.json" \
  --reward-json "Trajectory Generator/baseline_MLP/experimental_configs/morphology_completed_segment_shadow.json" \
  --output-dir validation/experimental_morphology_runs/2026-07-20_completed_segment_real_trace_replay
```

Validazione indipendente:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python \
  validation/validate_experimental_retrospective_morphology.py \
  validation/experimental_morphology_runs/2026-07-20_completed_segment_real_trace_replay/rollout_policy_trace.json \
  --output-dir validation/experimental_morphology_runs/2026-07-20_completed_segment_real_trace_validation \
  --strict
```

## Decisione consigliata e TODO

Stato consigliato:

```text
EXPERIMENTAL_SHADOW_APPROVED
ACTIVE_TRAINING_NOT_APPROVED
HARD_SAFETY_NOT_APPROVED
```

La scelta resta reversibile come richiesto:

- mantenere `event_anchored` se è necessario un termine causale per-step;
- selezionare la nuova modalità per analisi esatte post-segmento;
- tornare al legacy per confronti storici;
- continuare lo sviluppo senza cambiare il training corrente.

TODO prima di valutare una promozione:

- [ ] completare un rollout live shadow da 500 step della nuova modalità in
  una sessione OpenSim pulita e ripetere il confronto A/B;
- [ ] scegliere il design definitivo del bordo esterno phase-dependent e
  chiarire se la sicurezza debba essere immediata o retrospettiva;
- [ ] implementare e testare la riscrittura complete-segment/complete-episode
  delle reward prima del calcolo GAE;
- [ ] eseguire un pilot corto con peso piccolo soltanto dopo i gate precedenti;
- [ ] validare il pilot su start e seed held-out prima di qualsiasi training
  lungo o decisione di deployment.
