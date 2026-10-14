# Diagnosi dell'apprendimento RL39 LEFT

Stato: batch, update originale e replay A/B completati. È riprodotto un difetto
generale nei target GAE della versione Ray2.55.1 installata, corrispondente alla
[correzione ufficiale 59958](https://github.com/ray-project/ray/pull/59958).
Il controllo del replay ricostruisce esattamente actor, critic, Adam e tutti
gli 81 minibatch originali. Tutte le 13 valutazioni fisiche sono concluse:
12 a 500 passi, best80 rumoroso seed127 a 458 per penetrazione.
La [verifica finale](verification.json) è passata: 125 input iniziali invariati,
91 plot canonici, 12 dataset di rollout per il critic esterni al batch e
provenienza delle analisi verificati.

La correzione attenua la regressione nominale dell'update stock senza
recuperare la qualità iniziale: RMSE ankle 4,040° prima, 5,387° stock,
4,707° corretto. Il ritorno medio dei tre seed rumorosi è 235,375 prima,
240,997 stock e 272,142 corretto. Sono risultati descrittivi di un unico
aggiornamento; la forma delle coppie e la parità con V26 restano aperte.
Nessun teacher, modifica della reward o patch alle dipendenze installate.

Risultati: [report completo](<../../../../reports/user/2026-09-29_audit_RL_39d_bootstrap_e_aggiornamento_PPO.md>),
[confronto fisico corretto](corrected_physical_comparison/README.md),
[confronto dei target e dei critic](replay_comparison/report.md).

Il [protocollo](PROTOCOL.md) mantiene invariati reward, rete e governor e
separa esplorazione, obiettivo, stima dei vantaggi, aggiornamento PPO e risultati
fisici. Nessun comando teacher viene usato nel training.

## Riproducibilità

- `run_audit.py`: prepara configurazione e manifest SHA-256; ripristina il
  checkpoint completo best80, isola Ray e abilita la sola strumentazione runtime.
  Esegue esattamente una iterazione logica81. Rifiuta output preesistenti.
- `learner_probe.py`: registra episodi prima della GAE, batch dopo i connector,
  valori critic originali, minibatch, gradienti/delta dei parametri e snapshot.
  Le chiamate originali mantengono forward, loss, optimizer e RNG.
- `learner_analysis.py`: analisi offline del batch e degli snapshot; nessuna
  modifica della policy o raccolta di nuovi dati.
- `exploration_analysis.py`: ricostruzione raw/for-env e misure per fase e
  frammento. Le correlazioni temporali non sono effetti causali delle azioni.
- `evaluate.py`: nominale deployment e stocastico con sigma propria e connector
  training esatto; Gaussiani appaiati, critic inferito soltanto dopo il rollout.
- `run_evaluations.py`: organizza before/after e first_pass; massimo due processi
  fisici simultanei, watchdog per caso e riuso verificato del nominale best80.
- `physical_analysis.py`: cinematica, forma delle coppie, contatto, terminazioni,
  prefissi confrontabili e ritorni episodici conservati.
- `replay_batch.py`: controllo bitexact seguito dalla correzione dei soli target
  reali del batch, senza ambiente, nuova raccolta o aggiornamento delle dipendenze.
- `compare_replay.py`: confronto before/control/corrected su target stock e
  corretti, più confronto dei critic sugli stessi dati dei rollout disponibili.
- `run_corrected_evaluations.py`, `corrected_physical_analysis.py`: quattro casi
  aggiuntivi per la correzione, con manifest e risultati separati.
- `verify_upstream_equivalence.py`: confronto dei 4096 target reali corretti con
  il codice ufficiale Ray fissato al commit della correzione.
- `verify_results.py`: verifica finale degli input, artefatti, riproduzione
  stock e correttezza indipendente GAE, ricevute, stream Gaussiani e sette
  figure canoniche per ciascuno dei 13 casi.

Esecuzione usata: Python dell'ambiente `envCMC-rllib`, RLlib2.55.1, macOS arm64.
I percorsi nel codice usano pathlib e i launcher subprocess usano liste di
argomenti; la verifica live Windows resta aperta. Il processo principale è
avviato con priorità ridotta tramite `nice` su macOS; il codice non richiede
questo comando su Windows.

## Controlli iniziali

Tre test del probe passati: loss stock RLlib e aggiornamenti actor/critic/Adam
bitwise identici con/senza hook; RNG Python/NumPy/Torch invariati; GAE/target
uguali al connector installato, con terminazioni e truncation.
L'ultimo controllo attesta fedeltà dell'acquisizione, non correttezza della
formula installata: la ricorrenza indipendente ha successivamente rilevato
il difetto. Cinque test dell'analyzer e tre del replay includono riproduzione
dei connector nativi, target corretti e immutabilità delle colonne estranee.

Sei test esplorazione passati: allineamento nativo degli episodi con lookback,
assenza di correlazioni attraverso gap/episodi, segmenti terminali parziali,
stratificazione dei target e ricostruzione bitwise di `unsquash_action`.

Ripristino live verificato: 12 stati Adam ricollegati esattamente ai parametri,
step4455, learning rate0.0001, KL0.30000001192092896. Il valore0.2 nel JSON di
configurazione RLlib è quello di costruzione e non quello effettivo dopo restore.
Reward verificata su tutti i13 EnvRunner fisici; nessun critic rewarm.

Il tentativo iniziale fallito per import dei worker è conservato in
`attempts/01_startup_import`: zero transizioni raccolte e zero update.

La prima preparazione del replay si era fermata prima degli aggiornamenti
perché il restore generico arrotondava il learning rate a float32. Il replay
concluso in `boundary_replay_v2/` conserva esattamente i valori originali e
supera il controllo completo. Nessun risultato fisico dipende dal tentativo.

## Limiti da mantenere nel resoconto

Un batch e un aggiornamento descrivono il comportamento locale del best80, non
dimostrano capacità o impossibilità di apprendimento da zero. Tre seed di
valutazione sono uno screening appaiato, non una stima statistica robusta.
L'unico modello disponibile è AB06; LEFT/RIGHT non sono modelli differenti.
La sequenza offline già studiata dimostra fattibilità nominale, non che una
policy39 causale possa apprenderla o che abbia ritorno stocastico superiore.
