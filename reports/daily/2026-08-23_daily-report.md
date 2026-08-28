# Daily Report - 2026-08-23

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [Fase 0 — gate finale del recupero AB06](../user/2026-08-23_fase0_gate_finale_recupero_ab06.md)
- [F1/S1 — protocollo, tooling, dry-run e primo bridge F2R](../user/2026-08-23_f1_s1_protocollo_tooling_dryrun.md)

## Sintesi

La Fase 0 è stata chiusa con **PASS tecnico**: 58 job analizzati, provenienza content-addressed, overlay finali e deriva dell'attore verificati. Il verdetto scientifico sulla lineage B0820 è invece **FAIL come baseline AB06**: il collasso cinematico precede PPO, la trasformazione 39D→35D ha eliminato quattro feature privilegiate senza un refit target-domain sufficiente e il clock autonomo non riesce ad avviarsi in closed loop. I 50 update PPO non hanno ricostruito la forma biologica.

L'analisi isolata ha separato due cause. Con i target ricostruiti ma senza clock (`aiso_det`) non si completa alcun ciclo; con clock prescritto (`aiso_clk_diag`) si completano due cicli su tutti e tre gli start, dimostrando il **deadlock di bootstrap del clock**. È inoltre emersa un'ambiguità semantica della FSM v3 dopo la cancellazione di HS: i prodotti phase-aligned restano correttamente fail-closed finché il contratto non viene reso univoco.

È stato quindi preparato e auditato un bridge F2R. L'unico tentativo closed-loop T1R ha prodotto tre job tecnicamente validi, ma solo lo start nominale ha raggiunto 500 step; gli start −0,20 e +0,20 sono terminati per penetrazione a circa 28,8 mm. Il gate A è fallito, `unlock_t2=false`, quindi T2/T3, aggregazione e training sono rimasti bloccati. Nel gate B3, **−0,0099 rad non supera il requisito ≤ −0,03 rad**, perché −0,0099 è numericamente maggiore di −0,03.

## Lavoro svolto

- Completati matrice, overlay, actor drift e audit finale della Fase 0.
- Introdotti strumenti F1/F2R additivi per adapter teacher, raccolta, fit, receipt, gate e verifica no-clobber.
- Eseguite famiglie diagnostiche `aiso_det` e `aiso_clk_diag` per isolare l'effetto del clock.
- Preparato e sottoposto ad audit il protocollo T1R; eseguito una sola volta il rollout closed-loop autorizzato e poi il relativo gate.
- Preservati produzione, pesi di origine, plugin, modello, reward e semantica SEA; nessun PPO/ex-novo avviato.

## Test e verifiche

- F0 finale: **399 controlli PASS** e `py_compile` sui 19 file del perimetro.
- Matrice F0: 58 job censiti e verificati; overlay finale e actor drift con verifica strict PASS.
- T1R: **3/3 job tecnicamente validi**, ma gate A FAIL; nominale 500 step/2 cicli, start estremi terminati a 80 e 45 step per penetrazione.
- Contatori FSM v3 nel T1R: nessun resync, HS cancellato o timeout nei tratti eseguiti; il problema dominante è stato il contatto prima della chiusura dei cicli sugli start estremi.

## TODO completati

- [x] Chiudere tecnicamente la Fase 0 e produrre il verdetto comportamentale su B0820.
- [x] Provare causalmente il deadlock del clock tramite isolamento con clock prescritto.
- [x] Eseguire e fermare fail-closed il primo bridge T1R dopo il fallimento del gate A.

## TODO aperti e propagati

- [ ] Ridisegnare il bridge Teacher–Student/DAgger per riprodurre il metodo riuscito di luglio usando però esclusivamente il parent imitativo V26 di agosto.
- [ ] Correggere o rendere univoca la semantica `cycle_valid` dopo una cancellazione HS; mantenere fail-closed i prodotti phase-aligned fino alla risoluzione.
- [ ] Misurare sigma nel lineage corrente prima delle recovery; non assumere automaticamente 0,005.
- [ ] Risolvere insieme forma cinematica, bootstrap FSM/clock e margine di penetrazione closed-loop sugli start non nominali.
- [ ] Nei futuri run RLlib registrare separatamente runner configurati/sani, batch corrente, lifetime e replacement.
- [ ] Generalizzazione multimodello EPIC, differita rispetto al recupero AB06.
- [ ] TODO tecnici/amministrativi ereditati: swing timeout 1,3 s; driver di qualifica versionato; probe schema a `train_ppo_mlp.py:1408`; pin V20, `best == last`, `fit_p2` V12R3 e deflessione SEA iniziale.

