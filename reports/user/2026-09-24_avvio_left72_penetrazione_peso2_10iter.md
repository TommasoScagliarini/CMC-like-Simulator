# 2026-09-24 — Avvio continuazione LEFT72, penetrazione peso 2, dieci iterazioni

## Richiesta e stato

L'utente ha autorizzato le modifiche necessarie alla biforcazione del training
e l'avvio del solo ramo con peso di penetrazione 2. Avviato alle **11:36:33 CEST**,
dal checkpoint completo LEFT72 originale. Alle **11:38:13** la prima iterazione
aggiuntiva, **73**, è in esecuzione; il training non è ancora terminato.

Arresto previsto alla **82**: dieci aggiornamenti aggiuntivi da 4096 transizioni,
40.960 nuove transizioni complessive se il run conclude regolarmente. Sono
attivi 13 EnvRunner. La supervisione salva ogni iterazione e conserva i
milestone; `max_consecutive_skips=1` evita di proseguire saltando un aggiornamento
scaduto. Un timeout arresterebbe il run come incompleto.

Run:
`Trajectory Generator/unified_MLP_39/runs/training/MLP39_LEFT72_penetration_weight2_10iter_2026-09-24/`.
Piano, log di avvio e verifiche:
`Trajectory Generator/unified_MLP_39/runs/plans/left72_penetration_weight2_10iter_2026-09-24/`.
Supervisor PID 21137; processo trainer PID 21139 al controllo di avvio.

## Problema e soluzione

Il runtime ereditato dalla V1 rifiutava qualsiasi variazione della reward nel
resume. È stata aggiunta un'opzione esplicita,
`--resume-penetration-weight-override`, che consente di cambiare **soltanto**
`grf_penetration_weight` restando in modalità imitativa e usando una cartella
separata. Il normale resume conserva il controllo rigoroso della reward.

Il checkpoint del nuovo ramo registra configurazione aggiornata e provenienza
del cambiamento; può essere riaperto senza ripetere l'override. Actor, critic,
Adam e stato PPO/KL vengono ripristinati, senza reinizializzazione o warm-up
aggiuntivo. Il critic eredita inizialmente la stima della reward precedente e
si adatta durante i normali aggiornamenti PPO: non si assume che sia già
calibrato alla reward modificata.

L'audit controlla la reward nei wrapper effettivi di tutti i worker. Le
statistiche di ritorno ereditate dall'obiettivo precedente vengono eliminate
dal logger dell'Algorithm; i contatori di esperienza rimangono conservati.
Il manifest del checkpoint sorgente verifica che gli artefatti storici siano
rimasti immutati.

## Configurazione mantenuta

- Source: `milestone_iteration_000072/checkpoint_last` del run LEFT iniziato
  il 21 settembre; non il checkpoint 100 e non quello dello smoke.
- Unica variazione della reward: peso penetrazione **1 → 2**.
- Fix del contatto terminale hard già verificato e attivo.
- Prescribed sinistro guidato dalla fase sinistra, senza shift.
- Prescribed e clock visibili al 100%; divisore velocità 1.
- FSM v3, start unico, governor, soglie 20/28/30 mm e SEA conservati.
- PPO, esplorazione e struttura della rete uguali alla configurazione salvata
  della 72: batch 4096, minibatch 512, 10 epoche, LR 1e-4.
- Telemetria degli episodi attiva nella cartella `episode_telemetry/` del run.

La configurazione del controllo con peso 1 è pronta, ma **quel training non è
stato avviato**. Pertanto questo run da solo non completa il confronto A/B.

## Verifiche prima e dopo l'avvio

- **25 test passati**: regole della biforcazione, isolamento degli output,
  metadati persistenti, parametri ripristinati, worker, statistiche del logger,
  regressioni Adam/KL e contabilizzazione del contatto hard.
- **Smoke PPO reale** con 13 worker, 128 transizioni visibili, minibatch 64 e
  una sola epoca: un aggiornamento 72→73 in cartella separata.
- Audit dello smoke: actor e critic identici al source prima dell'aggiornamento,
  tutti i 12 stati Adam conservati (contatore iniziale 3807), KL conservato,
  tutti i worker con peso 2, nuova reward salvata nel checkpoint.
- Riapertura dello smoke in processo nuovo, override disabilitato e zero nuovi
  aggiornamenti: riuscita, con stato completo e provenienza conservati.
- **Audit del run effettivo PASS**: medesime verifiche sui parametri, Adam/KL e
  sui 13 worker, batch 4096, stop 82, codice corrispondente al manifest.

Ricevute: `preflight.json`, `smoke_verification.json`, `startup_audit.json`,
`source_checkpoint_hashes.json` e `training_source_hashes.json` nel piano.
Test/log sotto `validation/penetration_resume_2026-09-24/`.
Gli smoke restano in cartelle `runs/training/smoke_LEFT72_penetration_weight2_*`.
Verifiche live eseguite su macOS arm64; Windows non eseguito.

## File modificati

Dentro `Trajectory Generator/unified_MLP_39/`:

- Nuovo `reward_resume.py`: variazione consentita, provenienza e audit.
- `training_runtime.py`: integrazione nella ripresa e metadati aggiornati.
- `train_ppo_mlp.py`, `training_config.py`: opzione CLI/YAML esplicita.
- Nuovo `tests/test_reward_resume.py`.
- Configurazioni `configs/left72_penetration_weight{1,2}_10iter_2026-09-24.yaml`.
- Script `launch.py`, configurazioni degli smoke e artefatti nel piano indicato.

Nessuna modifica alla FSM, ai plugin o ai checkpoint storici. Il trascinamento
dei diagnostici di escursione nel reset FSM, descritto nel report precedente,
resta una questione separata e aperta.

## TODO

- [x] Preparare e verificare la biforcazione controllata.
- [x] Avviare il ramo peso 2 dalla 72 con arresto alla 82.
- [ ] Verificare dieci aggiornamenti riusciti, 40.960 transizioni e salute della
  telemetria alla conclusione; analizzare cinematica e robustezza a pesi fissi.
- [ ] Eseguire il controllo peso 1 se si procede con il confronto causale A/B.
- [ ] Valutare i candidati con perturbazioni comuni e misure fisiche, evitando
  di confrontare direttamente ritorni definiti da reward diverse.
