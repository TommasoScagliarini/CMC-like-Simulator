# Validation pipeline workspace

Questa directory è la sede canonica degli script, dei test e degli artefatti
specifici delle pipeline di rete/policy. Le implementazioni condivise di
training e runtime rimangono invece in `baseline_MLP` e nel simulatore root:
sono codice di produzione, non tooling di validazione.

## Pipeline V26C

La workspace storica J0–J21 è:

- `v26c_july_replica_2026-08-26/`

Runner, test, preregistrazioni, autorizzazioni, receipt, dataset, trace,
checkpoint e output della pipeline sono già collocati lì. La workspace è
sigillata: i gate J20/J21 verificano sia i percorsi sia gli SHA-256, quindi non
deve essere rinominata, riorganizzata o riscritta.

La chiusura esterna della pipeline è stata materializzata in:

- `pipeline_bundle_v26c_2026-08-28/`

Il bundle contiene oggetti fisici byte-identici di sorgenti, input, asset,
plugin, precedenti storici e report che in origine vivevano fuori da questa
directory. `bundle_manifest.json` collega ogni percorso originale al relativo
oggetto SHA-256, con ruolo, consumer e dimensione. `in_place_manifest.json` censisce
gli artefatti V26C e gli upstream F0/F1/F2R/V26B che erano già sotto
`validation`, senza duplicarne i 2,6 GB.

## Perché gli originali restano presenti

La pipeline V26C è content-addressed. Un move renderebbe i file mancanti; un
forwarder cambierebbe gli hash; un symlink non sarebbe affidabile su Windows.
Per questo il consolidamento è additivo:

1. il bundle sotto `validation` è la chiusura archiviata e verificabile;
2. gli originali restano ancore di compatibilità per import, pickle e pin, ma
   la verifica dell'archivio non dipende dalla loro presenza;
3. i report restano canonici in `reports/`, come richiesto da `AGENTS.md`, ma
   sono inclusi anche nel mirror di evidenza;
4. lo snapshot del codice production non deve essere usato come runtime
   canonico né modificato a mano.

## Regola per le nuove pipeline

Ogni nuovo runner, test, script di analisi e output specifico di una pipeline
deve nascere sotto questa directory. Il codice production condiviso può
restare fuori, ma la lineage deve registrarne una source closure byte-identica
e content-addressed. I nuovi runner devono individuare il repository tramite
sentinel (`AGENTS.md` e `Trajectory Generator`) e non mediante indici rigidi
come `parents[3]`.

Cache, bytecode, `.DS_Store` e log OpenSim incidentali non sono evidenza
scientifica e non entrano nei manifest.
