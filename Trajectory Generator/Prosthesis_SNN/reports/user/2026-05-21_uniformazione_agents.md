# Uniformazione di AGENTS.md

## Problema

Il file `AGENTS.md` conteneva istruzioni miste in italiano e inglese, con alcuni
refusi e convenzioni operative non completamente esplicitate. Questo rendeva il
file meno chiaro come fonte di riferimento per gli agenti che lavorano sul
progetto.

## Soluzione

`AGENTS.md` è stato uniformato in italiano e riorganizzato per rendere più
visibili:

- i vincoli fondamentali del progetto;
- il contratto del provider verso il simulatore;
- i file da preservare;
- le verifiche attese prima della consegna;
- la policy per i TODO;
- le istruzioni operative per i comandi `start_day`, `create_report` ed
  `end_day`.

## Strategia

La modifica è stata limitata alla documentazione operativa, preservando il
contenuto tecnico già presente:

- compatibilità Windows x86_64 e macOS arm64;
- core SNN portabile senza dipendenze runtime da OpenSim, Isaac Lab, skrl,
  Hydra o task specifici;
- uso di PyTorch e snntorch;
- integrazione futura tramite provider compatibile con
  `provider.get(t, state=None)`;
- primo scope SNN limitato a `pros_knee_angle` e `pros_ankle_angle`;
- divieto di modificare la semantica del plugin SEA C++ senza richiesta
  esplicita.

## File modificati

- `AGENTS.md`

## Test e verifiche eseguite

- Letto `AGENTS.md` dopo la modifica per controllare la struttura finale.
- Verificata l'assenza delle vecchie intestazioni inglesi principali.
- Non sono stati eseguiti test Python perché la modifica è solo documentale.

## TODO aperti

Nessun TODO aperto specifico per questa task.
