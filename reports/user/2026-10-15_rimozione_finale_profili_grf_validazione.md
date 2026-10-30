# 2026-10-15 — Rimozione finale dei profili GRF di validazione

Su richiesta dell'utente sono stati eliminati anche:

- `AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO_v4_mesh_experimental.json`
- `AB06_SEASEA_stiff321_500_pi_online_sensor_basis.json`

Sono 13.951 byte aggiuntivi rimossi, per un totale di 21 profili eliminati
dall'inventario iniziale di 24. Prima di cancellarli, entrambi sono stati
confrontati byte per byte con Git: il ripristino esatto resta possibile dal
commit `dba42388c32a441a5543f20881612a52781d7a99`.

## Strategia e modifiche

La rimozione riguarda esclusivamente i due JSON selezionati. Restano i tre
profili operativi: `grf_correct_magnitude`, `grf_detector_HS-TO` e
`online_full_wrench_residual_tangent_v2`, con il prefisso comune
`AB06_SEASEA_stiff321_500_pi_` e l'estensione `.json`.

Aggiornati i README di `online_grf_profiles/`, `Trajectory Generator/baseline_MLP/`
e `tools/online_grf_contact/`. L'esempio corrente di verifica del plugin usa ora
il profilo principale conservato; la descrizione dell'audit storico specifica
che utilizzava il vecchio sensor-basis. Nessun sorgente runtime, configurazione,
plugin, modello o checkpoint e stato modificato da questa operazione.

Test geometrici V4, smoke legacy e protocolli che selezionano i due file
eliminati richiedono ora il ripristino degli originali. I manifest e le
verifiche storiche non sono stati riscritti o indeboliti.

## Verifiche

Un subagente ha eseguito il controllo indipendente:

- esattamente tre JSON rimasti, tutti validi per il parser runtime;
- 31 riferimenti GRF in 22 YAML, incluso best47, risolti correttamente;
- nessun consumatore operativo esaminato seleziona i due profili eliminati;
- 37/37 test dei contratti runtime superati;
- 172 file di codice, configurazione, plugin e profili invariati nel controllo
  corrente; 54 file superstiti invariati anche rispetto alla baseline acquisita
  prima della precedente rimozione di 19 profili;
- `git diff --check` superato e collegamenti della documentazione aggiornati.

La cattura indipendente corrente e iniziata appena dopo l'unlink dei due JSON;
le prove di recupero Git sono state acquisite prima della cancellazione. Il
confronto con la baseline precedente copre anche lo stato anteriore a questa
operazione. Non e stata ripetuta una simulazione nativa: lo smoke best47 del
passaggio precedente rimane documentato separatamente, mentre qui sono stati
verificati tutti i riferimenti e gli hash delle sue dipendenze GRF.

Evidenze: [ricevute e log](../cleanup/2026-10-15_grf_final_removal/README.md).
Nessun commit o push eseguito.
