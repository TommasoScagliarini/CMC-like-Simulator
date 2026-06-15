# Diagnosi del crash nativo Ray e recovery dei training prolungati

**Data**: 2026-06-11  
**Contesto**: training PPO/MLP imitativo interrotto durante l'iterazione logica
22/40 in `Trajectory Generator/runs/baseline_mlp_imit_win`.

## Problema

Il training aveva completato regolarmente l'iterazione 21 e aveva iniziato la
22, ma si e' poi arrestato senza completare la run e senza essere rilanciato dal
wrapper di auto-restart.

Era necessario determinare:

1. se il crash fosse causato dalla rete, dalla reward, da OpenSim, da una
   divergenza numerica o dalla memoria;
2. perche' il meccanismo di recovery non avesse ripreso il training;
3. quale livello di affidabilita' fosse realistico aspettarsi dai prossimi
   training prolungati.

## Diagnosi

### Causa immediata confermata

La causa immediata del crash e' un collasso nativo del runtime Ray su Windows.

Alle `2026-06-11 11:02:42`:

1. il processo Ray `dashboard_agent` termina con exit code `1`;
2. il `raylet` termina perche' condivide il destino dell'agent
   (`dashboard_agent failed and raylet fate-shares with it`);
3. durante lo shutdown/disconnect compaiono ripetute
   `Windows fatal exception: access violation`;
4. il `raylet` registra inoltre uno stato interno inatteso:
   `object_manager_->SetErrorAll()` fallisce;
5. tutti i 12 EnvRunner vengono persi perche' il nodo Ray locale e' crashato.

Le access violation avvengono dentro il codice Ray installato, in particolare
nel percorso:

```text
ray._private.worker.job_logging_config
ray._private.worker.disconnect
ray._private.worker.shutdown
```

### Cause escluse dai log

Non risultano evidenze di:

- memoria esaurita: Ray registra esplicitamente `task failed due to oom: 0`;
- timeout dell'iterazione: il timeout configurato era 3600 s e non era scaduto;
- divergenza della policy o del critico;
- errore della reward;
- errore deterministico di OpenSim o del simulatore biomeccanico.

L'iterazione 21 era sana:

```text
logical_iteration = 21
episode_return_mean = 21.393156428901886
vf_explained_var = 0.7522317171096802
checkpoint_last = valido
```

L'iterazione 22 e' quindi il punto temporale in cui Ray e' crashato, non una
iterazione identificata come patologica per il training.

### Causa radice non ancora identificata

I log non spiegano perche' il `dashboard_agent` sia terminato. Ray stesso elenca
tra le possibili cause una versione di `grpcio` incompatibile, un errore
dell'agent o una terminazione da parte del sistema operativo.

L'ambiente osservato usa:

```text
Python   3.12.13
Ray      2.55.1
grpcio   1.81.0
protobuf 7.35.0
torch    2.7.0
```

Questa combinazione potrebbe contribuire all'instabilita' nativa su Windows,
ma i dati raccolti non consentono di attribuire con certezza il crash a
`grpcio`, `protobuf` o a una specifica incompatibilita' binaria.

Un precedente run era gia' crashato all'iterazione 14 con altre access violation
native Ray. Il fatto che i due crash siano comparsi in punti diversi suggerisce
un problema sporadico del runtime Ray/Windows, non un errore deterministico
legato a una precisa iterazione o a uno specifico batch.

## Secondo problema trovato: recovery incompleto

Il crash nativo Ray non era l'unico problema operativo.

Il supervisore Python rilanciava il training solamente quando il child
terminava con:

```text
stop_reason = iteration_timeout
```

Un access violation puo' invece terminare il processo prima che questo aggiorni
`summary.json`. Il supervisore leggeva quindi una summary vecchia e terminava
senza ripartire da `checkpoint_last`.

Anche il wrapper PowerShell esterno poteva restare bloccato dentro
`conda run --no-capture-output`, senza raggiungere il tentativo successivo.

Questi difetti non hanno causato l'access violation, ma hanno trasformato un
crash sporadico recuperabile nell'arresto definitivo del training.

## Soluzione implementata

Il recovery e' stato internalizzato nel supervisore Python di
`train_ppo_mlp.py`.

Ora il supervisore:

- registra il timestamp di `summary.json` prima di avviare ogni child;
- riconosce una summary non aggiornata come uscita nativa/non riportata;
- riconosce anche le uscite child non-zero e gli errori Python espliciti;
- termina gli eventuali discendenti Ray rimasti;
- verifica che `checkpoint_last` e `checkpoint_last_meta.json` siano validi;
- riparte dall'ultimo checkpoint completo;
- ritenta la stessa iterazione interrotta, senza marcarla come completata o
  saltata;
- registra cronologia e conteggio dei crash in `summary.json` e
  `supervisor_state.json`;
- interrompe il run dopo troppi crash senza avanzamento del checkpoint, evitando
  loop infiniti.

Nuovo parametro CLI:

```text
--max-consecutive-crash-restarts
```

Il default e' `5`.

## Giudizio operativo sui training lunghi

Non e' ancora possibile garantire che un training prolungato rimanga
ininterrotto: esiste un rischio reale di ulteriori crash nativi Ray la cui causa
radice non e' stata identificata.

Il nuovo recovery cambia pero' il rischio operativo:

| Aspetto | Stato |
|---|---|
| Garanzia di training senza crash | Non disponibile |
| Recupero da crash sporadico tra due checkpoint | Implementato |
| Ripresa dalla stessa iterazione interrotta | Implementata |
| Perdita massima attesa con `--checkpoint-every 1` | Iterazione in corso |
| Protezione da crash ripetuti senza progresso | Implementata |
| Stabilita' del runtime Ray/Windows | Da validare |

Pertanto, un training lungo puo' essere considerato **recuperabile**, ma non
ancora **intrinsecamente stabile**.

Restano scenari residui che possono comunque arrestare il training:

- crash ripetuti oltre la soglia configurata;
- crash durante la scrittura del checkpoint;
- checkpoint mancante o corrotto;
- incompatibilita' nativa persistente del runtime Ray.

## Strategia seguita

1. Ricostruzione della cronologia tramite checkpoint, heartbeat, summary e
   metriche per iterazione.
2. Analisi dei log della sessione Ray:
   `raylet.out`, `raylet.err`, `gcs_server.out`, log del driver e agent.
3. Esclusione di OOM, timeout e divergenza del training.
4. Confronto con il precedente crash nativo avvenuto all'iterazione 14.
5. Review del supervisore Python e del wrapper PowerShell.
6. Implementazione del recovery per uscite native/non riportate.
7. Validazione deterministica tramite crash simulati.

## File modificati

```text
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/README.md
```

Nessuna modifica al plugin C++ SEA, alla semantica del comando SEA, alla reward
o al simulatore biomeccanico.

## Verifiche eseguite

- Analisi del run reale: checkpoint valido fino all'iterazione 21 e heartbeat
  fermo su `algo.train iteration 22`.
- Analisi Ray: uscita `dashboard_agent`, fate-sharing del `raylet`, access
  violation native e perdita del nodo confermate.
- Verifica OOM: nessun task fallito per memoria.
- `py_compile` di `train_ppo_mlp.py`: PASS.
- `git diff --check`: PASS, esclusi i normali warning LF/CRLF.
- Parsing CLI `--help`: nuovo flag presente.
- Probe di crash nativo simulato alla 22:
  ripresa dal checkpoint 21 e nuovo tentativo della 22: PASS.
- Probe con `summary.json` vecchia, equivalente al run reale:
  rilevazione `unreported_exit` e recovery: PASS.
- Probe con crash ripetuti senza avanzamento:
  arresto alla soglia anti-loop: PASS.
- Artefatti temporanei dei probe rimossi.

Non e' stato eseguito un nuovo training lungo reale con il supervisore
aggiornato; questa rimane la verifica operativa principale ancora necessaria.

## TODO

- [ ] Eseguire un training prolungato con il nuovo supervisore,
      `--checkpoint-every 1` e `--max-consecutive-crash-restarts 5`, verificando
      recovery e avanzamento dopo eventuali crash reali.
- [ ] Stabilizzare e validare l'ambiente Ray su Windows, valutando una matrice
      controllata di versioni Python/Ray/`grpcio`/`protobuf` senza modificare
      l'ambiente produttivo alla cieca.
- [ ] Raccogliere un dump nativo piu' informativo del prossimo access violation
      per identificare la causa radice del `dashboard_agent` crash.
- [ ] Verificare fisicamente su macOS arm64 il cleanup del process group e il
      resume RLlib, TODO propagato dal 2026-06-10.
- [ ] Durante il prossimo training monitorare anche `vf_explained_var` e il
      divario predetto-vs-return, TODO propagato dal report
      `2026-06-11_monitoraggio_critico_vf_metrics_prossimo_training.md`.
