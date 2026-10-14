# Continuazione RIGHT39 dalla 40 alla 60: preparazione

Preparata il 23 settembre 2026; **non avviata da questa preparazione**. Il processo
principale decide quando eseguire `--run`, dopo lo screening rumore già autorizzato.
Il launcher non avvia né attende autonomamente altri esperimenti.

Comando di esecuzione, dalla root del repository:

```sh
/opt/anaconda3/envs/envCMC-rllib/bin/python -u 'Trajectory Generator/unified_MLP_39/validation/right_reference_resume60_2026-09-23/run_60.py' --run
```

Per persistenza oltre il turno, avviare quel comando con `subprocess.Popen`,
`stdin=DEVNULL`, log `monitor.log` nel plan e `start_new_session=True` su macOS;
su Windows usare il flag di creazione di un nuovo gruppo di processo. Il codice
non contiene un percorso Python hardcoded: il comando qui sopra identifica solo
l'interprete dell'ambiente macOS attuale. Il trainer figlio usa `sys.executable`.

Controllo senza training:

```sh
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/unified_MLP_39/validation/right_reference_resume60_2026-09-23/run_60.py' --check-only
```

Il piano è `unified_MLP_39/runs/plans/right_reference_resume60_2026-09-23/`.
`config_40_to_60.yaml` è identico alla configurazione risolta del checkpoint40,
eccetto `simulation.iterations: 60`. `--resume-from` viene passato esplicitamente
alla CLI, indicando `milestone_iteration_000040/checkpoint_last`.

Il run rimane `MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23`: **il nome
della cartella conserva 40iter**, ma lo storico verrà esteso fino alla 60. Si
aggiungono 20 iterazioni e 81.920 transizioni, per un totale nominale di 245.760.
FSM v3, riferimento RIGHT, clock visibile, sei ingressi disponibili100%, PPO,
13 worker/14 CPU Ray e limiti fisici restano invariati. Nessun oscuramento.

## Verifiche già eseguite

- `preflight.py` costruisce soltanto il learner locale dal checkpoint: zero Ray,
  ambienti, passi simulati o update. Nessun file del checkpoint viene cambiato.
- Tutti i pesi actor/critic ripristinati coincidono esattamente con la sorgente.
- I 12 stati Adam salvati sono ripristinati identici e collegati ai parametri
  vivi; contatore degli update **3240**. La correzione di produzione gestisce
  il noto problema RLlib dei parameter ID tensorizzati.
- KL **0,20000000298** e scheduler costanti ripristinati dal sidecar
  `unified_learner_state.json`; ulteriori campi learner invariati.
- `--prepare` e `--check-only` superati; `execution.lock` assente alla preparazione.
- Congelati **145 input** e **36 file del milestone40**, con snapshot dei sorgenti.
  Nessuno script o risultato dello screening corrente è congelato.
- `pre_resume_40/` conserva 61 file: metadati/log root del run, storico, TensorBoard,
  modulo iniziale, milestone40 completo e file originali del piano di lancio.

## Supervisione e arresto

Il monitor scrive `status.json` ogni cinque secondi e `launch.json` all'avvio.
Su macOS `caffeinate -i -w <supervisor_pid>` resta attivo soltanto per la durata
del training. `execution.lock` impedisce un secondo avvio accidentale di questo piano.

Controlla la conservazione byte per byte del prefisso di storico1–40; ogni nuova
iterazione deve essere contigua, entro41–60, con4096 transizioni visibili, loss
finite, aggiornamenti di media/critic e telemetria completa. Alla41 confronta
i digest actor/media/critic con la40, verifica gli audit live Adam e PPO e scrive
`resume_verified.json`. Non basta il solo preflight offline per certificare la
ripresa del processo effettivo.

Il trainer riceve il limite60 e il monitor verifica il completamento esatto,
senza iterazioni saltate. Dopo l'uscita controlla gli update Adam nei checkpoint
40/41/60, registra i digest del60 e termina. **Non avvia rollout, altri checkpoint
di valutazione o continuazioni a80/100.**

Se un audit fallisce il monitor interrompe il figlio e registra `failed`;
conserva lock e artefatti per l'ispezione, senza una ripartenza automatica del
monitor. Il supervisore standard del trainer mantiene i suoi watchdog; un crash
restart viene evidenziato dall'audit finale e non autorizza lavoro oltre60.

La ripresa conserva lo stato di apprendimento, ma gli episodi fisici e lo stato
casuale dei processi non ripartono bit per bit dal punto intermedio. Può esserci
un transitorio alla41. Inoltre il monitor non decide quando lo screening abbia
liberato le risorse: il processo principale lo avvia solo dopo la chiusura delle
simulazioni. La durata indicativa delle20 iterazioni è circa2h45 sulla base del
run appena concluso, senza garanzia sui tempi futuri.
