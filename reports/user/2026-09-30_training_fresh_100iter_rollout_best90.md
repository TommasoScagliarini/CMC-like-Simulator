# 2026-09-30 — Training LEFT da zero a 100 iterazioni e rollout del best90

> Aggiornamento del 2026-10-17: alcuni materiali citati sono stati rimossi
> nella pulizia del progetto. I collegamenti contrassegnati «materiale
> rimosso» portano al registro con il percorso originale.


Instruction check token: `CMC_AGENT_OK_2026`.

## Problema e obiettivo

Best80 LEFT, con peso penetrazione 2, era stato ottenuto ripartendo dal checkpoint LEFT72 e applicando otto aggiornamenti dopo il cambio del peso da 1 a 2. Questo risultato non dimostrava che la configurazione finale fosse in grado di apprendere da zero.

Su richiesta dell’utente è stato avviato il 29 settembre un training nuovo fino a 100 iterazioni. Il 30 settembre sono stati eseguiti il rollout deterministico del best e la generazione dei sette plot canonici. Su indicazione esplicita dell’utente del 30 settembre, best90 fresh è la nuova baseline corrente; best80 resta il riferimento storico.

## Soluzione e strategia

La configurazione è stata derivata dalla `training_cfg.resolved.yaml` del run sorgente best80, conservando impostazioni fisiche, reward, architettura e parametri PPO. Il confronto delle sezioni ha verificato che le differenze riguardano il limite di iterazioni, l’identità del nuovo run e la provenienza.

- Actor e critic inizializzati da zero; ottimizzatore e stato PPO nuovi, senza restore o trapianto dell’actor.
- Peso penetrazione 2 dall’inizio; nuovi costi di tuning ankle/knee/jerk a zero.
- Target LEFT e clock prescribed visibili, FSM v3, actor a 39 feature.
- Due hidden layer da 256, attivazione tanh, actor/critic asimmetrici.
- Seed 123, batch 4096, minibatch 512, 10 epoche, learning rate 1e-4.
- 13 worker, episodi di 5 s, segmenti di 0,01 s, offset iniziale di 1 s.
- Coefficiente KL iniziale configurato a 0,2, con adattamento PPO normale. Non è stato ereditato il valore adattato del checkpoint best80.
- Checkpoint conservati a ogni iterazione e telemetria degli episodi abilitata.

La verifica CLI ha confermato 100 iterazioni, `resume_from=null` e nessuna inizializzazione da actor esistente. Il training è stato avviato in un processo persistente separato. Su macOS il launcher usa `caffeinate`; contiene anche il ramo di creazione processo Windows, ma non è stato provato live su Windows.

## Esito del training

Fonte: [summary del training — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-07181fe70ed86880>).

| Voce | Risultato |
| --- | --- |
| Avvio | 29 settembre 2026, 10:10:20 CEST |
| Fine | 29 settembre 2026, 23:21:01 CEST |
| Tempo riportato dal summary | 47.438,85 s, circa 13 h 11 min |
| Iterazioni completate | 100/100 |
| Inizializzazione | `fresh` |
| Iterazioni saltate | Nessuna |
| Riavvii / crash restart | 0 / 0 |
| Arresto | `completed` |
| Best | Iterazione logica e RLlib 90 |
| Miglior ritorno medio di training | 274,4460 |

Il best è quello selezionato dal criterio di training, non una selezione comparativa già conclusa sulla qualità fisica o sulla robustezza. Last100 non è stato valutato in questa attività.

## Rollout deterministico del best90

Il rollout carica `rl_module_best` e la configurazione salvata nel run, con azione deterministica, registrazione degli output OpenSim e della traccia policy. Non vengono svolti aggiornamenti PPO.

Fonte: [rollout summary — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-db376ebb13abed03>).

| Metrica | Valore |
| --- | ---: |
| Durata | 5 s, da 12,99 a 17,99 s |
| Step | 500 |
| Ritorno deterministico | 301,1400 |
| Reward media | 0,60228 |
| Cicli FSM validi | 2 |
| HS / TO validi | 3 / 4 |
| Eventi invalidi | 0 |
| Penetrazione massima registrata | 17,524 mm |
| Fine episodio | `episode_time_limit` |
| `terminated` / `truncated` | `false` / `true` |

Il rollout completa l’orizzonte senza terminazione anticipata. Il ritorno del rollout deterministico e quello medio di training hanno modalità di raccolta diverse e non vanno interpretati come la stessa misura.

Il resolver della CLI ha collocato gli output nel percorso annidato `runs/rollout/unified_MLP_39/runs/rollout/` indicato dal collegamento sopra. Gli artefatti sono stati conservati in quel percorso effettivo; non è stato modificato il resolver in questa attività.

## Sette plot canonici

Cartella: [09_30_2026_unified39_LEFT_fresh_best90](../../plot/09_30_2026_unified39_LEFT_fresh_best90/).

1. `01_time_sea_control_reserve.png`
2. `02_time_joint_motor_states.png`
3. `03_gaitcycle_torque_angle_power.png`
4. `04_gaitcycle_joint_velocity_power.png`
5. `05_time_tau_input_tracking_error.png`
6. `06_time_joint_ref_sea_error.png`
7. `07_mlp_policy_vs_sound_leg_error.png`

È stato riutilizzato `validation/render_canonical_rollout.py`. La figura 7 confronta la reference servita con il target LEFT registrato; la gamba sana destra nella riga centrale è contesto. Il nome storico del PNG non cambia la provenienza del target.

La cartella generica iniziale `plot/09_30_2026_1` è stata rinominata con il nome del rollout e le ricevute sono state aggiornate. I PNG risiedono sotto la root `plot/`; nella cartella del rollout sono conservati soltanto ricevuta e log di rendering.

## File creati e modificati

Tutti i percorsi abbreviati seguenti sono sotto `Trajectory Generator/unified_MLP_39/`.

- [Configurazione fresh — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-081f13b1df694839>).
- [Piano e launcher — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-b5d44d81c7824dc8>): `config.yaml`, `preflight.json`, `launch.py`, `launch.json`, `training.log`.
- [Run di training — materiale rimosso](<../cleanup/2026-10-17_trajectory_removal/README.md#artifact-8b79ed9dccd92cfa>): checkpoint, configurazione risolta, summary e telemetria.
- Rollout e trace nel percorso effettivo collegato sopra, con `rollout_execution.log` e `plots/render.log`.
- Sette PNG, `plot_receipt.json`, `verification.json` e diagnostica dei canali nella cartella root dei plot.
- Il presente report.

Nessuna modifica al codice del simulatore, al controller, al plugin SEA, alla FSM o alla reward è stata necessaria per queste esecuzioni. Gli artefatti best80 sono conservati.

## Test e verifiche eseguite

- Confronto della configurazione con quella risolta del run best80 e controllo CLI del percorso fresh.
- Verifica dell’avvio effettivo della prima iterazione il 29 settembre.
- Lettura del summary finale e dei metadati best: 100 iterazioni concluse, best90, nessun riavvio.
- Rollout concluso con exit code 0 e `ok=true`; registrati tutti i 500 step.
- Rendering concluso con exit code 0; messaggio `No missing channels`.
- Verificati esattamente sette PNG, leggibilità con Pillow e hash SHA-256 salvati in [verification.json](../../plot/09_30_2026_unified39_LEFT_fresh_best90/verification.json).
- Ispezione visiva della figura 7: target LEFT e legende coerenti.

Non è stata rilanciata la suite di test del codice, perché l’attività usa la pipeline esistente senza modificarla. Le verifiche live attestano macOS arm64.

## Interpretazione e TODO

Il training da zero ha prodotto un checkpoint che completa il rollout nominale di 5 s. Questo risolve la verifica operativa di fattibilità per questo seed, ma non dimostra superiorità su best80, equivalenza delle coppie o robustezza tra seed. Per decisione esplicita dell’utente, la baseline corrente è best90 fresh: `rl_module_best` del run `MLP39_LEFT_fresh_best80config_100iter_2026-09-29`, selezionato all’iterazione 90. Best80 resta il riferimento storico per il confronto ancora aperto; tale confronto non è una condizione per adottare la nuova baseline.

- [ ] Confrontare best90 fresh con best80 sulla stessa finestra e con metriche comuni: tracking per giunto/fase, contatto, cicli, impulso, area positiva della coppia ankle e regolarità knee.
- [ ] Valutare la robustezza con perturbazioni comuni a policy congelate; distinguere seed di valutazione e repliche indipendenti di training.
- [ ] Eseguire verifica live Windows della pipeline scelta.

Restano aperti i TODO del [daily del 29 settembre](../daily/2026-09-29_daily-report.md) non risolti da questa attività, tra cui diagnosi del KL nei pilot critic-only, sufficienza del riadattamento critic, tuning separato knee/cinematica e successivi piani di oscuramento/ex-novo. Questo run fresh non include critic-only e non isola quelle ipotesi. La chiusura giornaliera dovrà propagare questi TODO insieme ai precedenti ancora validi.
