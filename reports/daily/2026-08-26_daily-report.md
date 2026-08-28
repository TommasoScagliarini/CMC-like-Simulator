# Daily Report - 2026-08-26

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [V26C J0 — audit e preregistrazione della replica July-faithful](../user/2026-08-26_v26c_j0_audit_preregistrazione_replica_luglio.md)

## Sintesi

È stata ricostruita e preregistrata la pipeline di luglio come metodo, mantenendo il **parent operativo esclusivamente V26 imitation agosto**. La ricostruzione ha mostrato due stadi distinti: BASE su 500 coppie teacher con fit dell'intera mean actor, seguito da MARKOV con anchor auto-distillate, recovery student-visited e multistart teacher. Il nuovo lineage usa un unico actor **35→256→256→4**; nello stadio base le colonne clock 0–1 e controller-memory 25–34 sono hard-zero, poi le colonne 25–34 vengono riabilitate nello stesso actor.

Il preflight J0 ha anche rilevato che il builder storico non propagava tutte le chiavi FSM v3/morphology: J1 non può quindi riusarlo implicitamente e deve costruire l'environment corrente con configurazione completa e verifiche fail-closed. Sigma è rimasta esplicitamente non assunta e fuori scope fino allo stadio recovery.

Il verdetto è stato **GO-FOR-ARCHITECT-REVIEW**: nessun fit, rollout, dataset o training è stato eseguito in J0.

## Lavoro svolto

- Ricostruiti da codice/artefatti i parametri BASE e MARKOV di luglio.
- Verificata la lineage: nessun peso, dataset o trace luglio come input operativo.
- Creati audit, test e receipt J0 in `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/`.
- Definito il contratto del futuro collector J1 con runtime V26, detector corrente, FSM v3 e Morphology Corridor completo.
- Conservati LOTO/LOCO/B1R1/B1R2 come TODO futuri, fuori dalla replica.

## Test e verifiche

- `test_v26c_j0_audit.py`: **156 check PASS**.
- Audit read-only e receipt no-clobber completati.
- Produzione, FSM, GRF, morphology, reward, SEA e plugin C++ invariati.

## TODO completati

- [x] Ricostruire i due stadi luglio senza usarne gli artefatti come parent.
- [x] Fissare un unico actor 35D senza adapter e senza widening.
- [x] Identificare l'inadeguatezza del builder storico rispetto al runtime v3/morphology corrente.

## TODO aperti e propagati

- [ ] Dopo approvazione, implementare ed eseguire J1 teacher collection con builder full-config e gate fail-closed.
- [ ] Eseguire J2 BASE fit e J3 qualifica closed-loop prima di aprire MARKOV/recovery.
- [ ] Misurare sigma nel lineage corrente prima di J4; non ereditare automaticamente 0,005.
- [ ] Applicare e registrare la policy di penetrazione corrente; le soglie verranno poi consolidate in 20 mm soft, 25 mm diagnostica luglio e 28 mm hard.
- [ ] Conservare LOTO/LOCO/B1R1/B1R2 come TODO non operativi.
- [ ] Generalizzazione multimodello EPIC e debiti tecnici/amministrativi ereditati ancora aperti.

