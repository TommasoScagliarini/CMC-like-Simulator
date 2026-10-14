# Correzione del reader V2 — criteri invariati

Data: 2026-10-13. La preregistrazione scientifica resta byte-identica:
`classification_preregistration.json`, SHA256
`4dd250b2a2c1d9b6f85c54cc994f6e2df4ac2734c6bc3c02196a04d709fbf0d2`.
Nessuna soglia, denominatore, campione, reward, harness o policy è cambiato.

La prima versione leggeva receipt e summary per classificare la causa finale.
Il root ha segnalato un ritorno terminale con `end_reason` morphology che
nascondeva un guard di penetrazione nativo documentato nel journal restituito.
Quel ritorno aveva lo stesso tempo logico del precedente: una chiamata `step`
aveva restituito reward/terminale senza completare il segmento di integrazione.
V1 avrebbe potuto classificarlo come postprocessing irrisolto e rifiutare il
timestamp duplicato, pur essendo presente evidenza fisica del guard.

La correzione è limitata alla lettura:

- `returned_rewards.jsonl.failure` e il `terminal_contact` primario
  strutturato sono ulteriori fonti della causa. Penetrazione, limite, sample
  fisico e snapshot devono concordare; non si inventa una misura dal solo testo.
- Un guard primario testimoniato resta `PHYSICAL_FAILURE`, anche se il summary
  riporta una causa morphology o gli output successivi sono incompleti.
- Una risposta terminale zero-duration è permessa soltanto come ultima
  risposta effettivamente registrata e accompagnata da failure/contact.
  Rimangono distinti numero di return/reward e segmenti con durata positiva.
- Il reward terminale effettivamente restituito resta nella contabilità.
  Il payload FSM invariato di tale risposta non vale come nuovo evento/ciclo.
- I sample nativi del segmento interrotto sono conservati e contati
  separatamente; non diventano reward, eventi o RMSE del prefisso completato.
  Il picco di penetrazione considera anche il contatto terminale misurato.

V1 e il suo seal restano in posizione originale e sono copiati byte per byte
in `classifier_v1_archive/`, con manifest degli hash. La nuova implementazione
è `aggregate_results_v2.py`, con test e seal separati. Tutti i17 test originali
e tre nuovi casi sintetici terminali passano. I nuovi test coprono causa
mascherata, return terminale senza avanzamento, reward32/segmenti31, duplicazione
del journal FSM evitata e rifiuto di zero-duration non terminale.

Il codice V2 viene sigillato prima della lettura diretta dei file del caso
segnalato. Una successiva verifica saved-output potrà confermare il caso, senza
rerun o sostituzione della posa e senza reinterpretare come PASS il failure.
