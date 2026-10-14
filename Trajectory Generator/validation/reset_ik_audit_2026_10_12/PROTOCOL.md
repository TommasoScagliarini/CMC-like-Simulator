# Protocollo reset IK — 12 ottobre 2026

Autorizzazione: indagine e test richiesti dall'utente, audit indipendente Claude
Opus 5.5 effort max e successiva sintesi; `create_report` richiesto nella stessa
sessione. Instruction check token: CMC_AGENT_OK_2026.

## Domande e ambito

1. Confermare l'origine di q/qdot al reset, l'inizializzazione di SEA, governor
   e FSM e la distribuzione effettiva degli start del training V26C best47.
2. Distinguere prescribed negli ingressi diretti dell'actor, nel critic,
   nell'ambiente biologico, nelle GRF e nella reward.
3. Misurare la ripetibilita dei reset e il comportamento della stessa policy
   congelata con start addestrati e nuovi, e velocita iniziali protesiche nulle.

Nessuna modifica alla produzione, al plugin, ai pesi o ai profili operativi.
Il banco primario e AB06, setup e config del run
`MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01`, `rl_module_best`.
L'actor e 35D e il critic legge l'osservazione completa 84D.

## Test prima della lettura degli esiti

- Reset ripetuti con seed diversi e start fissato: q/qdot, SEA, memoria
  governor/FSM, osservazione e prima azione; verificare identita entro precisione
  numerica e differenze reali fra start.
- Inferenza deterministica seed123, 5s per caso, integrazione e criteri normali:
  nominale; due start addestrati ±0.2s; fase non addestrata +0.4s; successivo HS
  LEFT misurato; velocita iniziali protesiche nulle mantenendo la posa nominale
  e motor_speed coerente. In quest'ultimo caso il governor conserva la propria
  condizione IK originale: e una perturbazione della velocita fisica a parita
  di riferimento iniziale, non un reset con tutte le memorie riallineate.
- Il test a velocita nulla in una posa di cammino NON e avvio da una posa
  statica bilanciata: il TODO V26-T04 non viene dichiarato risolto.

Metriche: tempo/steps/causa di fine; cicli ed eventi FSM; tracking actual-served
per giunto; escursioni; penetrazione; carico/reserve; clipping e condizioni
iniziali. Startup primo secondo e regime successivo distinti quando coperti.
Un fallimento resta nell'analisi; nessuna ripetizione selettiva per promuovere
il risultato. Eventuali errori tecnici del harness vengono conservati e
corretti senza modificare dinamica/limiti del banco.

Esecuzione in processi isolati, fino a3 paralleli sullo stesso host; questo
riduce il tempo di attesa e non cambia il tempo fisico della simulazione.
Output/artifact di audit sotto questa cartella. Sette plot canonici per rollout
nella sottocartella dedicata di `plot/` al root, anche per rollout interrotti.
Hashes di sorgenti, setup, dati, modelli, plugin e checkpoint conservati.

## Interpretazione e indipendenza

Una policy congelata consente di misurare sensibilita e copertura degli start;
non identifica causalmente quanto cambiare il reset migliorerebbe il training.
Una simile attribuzione richiede un confronto PPO paired a budget, inizializzazione,
reward e seed comuni, con distribuzione reset come unico fattore variato e
valutazione held-out. Nessuna diagnosi su questo punto viene dedotta dal solo
completamento temporale o da ritorni diversi.

L'audit source di Claude legge direttamente sorgenti e log storici senza
conclusioni Codex o risultati dei nuovi test. Codex sigilla il proprio audit
source prima di leggere l'output Claude. Dopo i test Claude riceve entrambi
gli audit e gli artifact numerici, per confronto e critica della sintesi.
Modello effettivo e effort richiesto sono registrati; nessun fallback ammesso.
Le esecuzioni native di questa macchina attestano macOS arm64; il codice
del harness deve restare utilizzabile su Windows con l'interprete OpenSim.
