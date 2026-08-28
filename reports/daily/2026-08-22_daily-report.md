# Daily Report - 2026-08-22

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- [Fase 0 — freeze, inventario e gate intermedio](../user/2026-08-22_fase0_freeze_inventario_gate_intermedio.md)
- [TODO — generalizzazione multimodello con EPIC](../user/2026-08-22_todo_generalizzazione_multimodello_epic.md)

## Sintesi

È stata avviata la Fase 0 del recupero della qualità AB06: freeze degli artefatti, inventario, ricostruzione della provenienza e preparazione della matrice di rollout comparabili. La revisione r3 ha chiuso il cross-check degli artefatti risolvibili con **23 PASS, 0 FAIL e 10 N/A motivati**; al termine del giorno il gate era ancora intermedio e il documento è stato poi formalmente superato dal gate finale del 23 agosto.

È stato anche chiarito il dubbio sul numero di environment del training B0820: non vi era un numero di environment in crescita. La configurazione usava **12 remote EnvRunner** e un batch effettivo di **4608 step = 12 × 384**; il valore crescente osservato era `num_env_steps_sampled_lifetime`, cioè un contatore cumulativo. Il valore 4096 apparteneva ad altre configurazioni, tra cui il training imitativo V26.

La generalizzazione è stata separata dal recupero AB06. Teacher, dati, IK/GRF, dinamica e Morphology Corridor correnti rendono la policy statisticamente dipendente da AB06; il metodo Teacher–Student/DAgger non è però intrinsecamente limitato a quel modello. È stato aperto un TODO dedicato che richiede una validazione cross-subject su un modello EPIC mai usato per teacher, DAgger, PPO o selezione degli iperparametri.

## Lavoro svolto

- Costruito il perimetro additivo `Trajectory Generator/baseline_MLP/validation/f0_freeze_2026-08-22/` per inventario, source closure, matrici, receipt e analisi fail-closed.
- Conservate immutabili le revisioni F0 superate e gli artefatti utente già presenti.
- Eseguite le famiglie iniziali `replay` e `det`; `stoch` e `ctrl39` erano ancora sospese a fine giornata.
- Formalizzato in un report autonomo il futuro processo di generalizzazione multimodello con dataset EPIC.
- Nessun peso, checkpoint, reward, plugin C++, modello OpenSim o semantica SEA è stato modificato; nessun training è stato lanciato.

## Test e verifiche

- Cross-check inventario r3: **23 PASS / 0 FAIL / 10 N/A**.
- Replay iniziali: **9/9**; rollout deterministici iniziali: **19/19**.
- Test progressivi su receipt, provenienza, ABI, chiusura runtime e no-clobber descritti nel report F0; output già prodotti verificati byte-identici durante le revisioni.

## TODO completati

- [x] Chiarire il significato dei contatori RLlib e del batch 4608 rispetto a 4096.
- [x] Separare formalmente il recupero AB06 dalla generalizzazione multimodello.
- [x] Congelare e censire gli artefatti necessari alla Fase 0.

## TODO aperti e propagati

- [ ] Completare l'audit F0, le famiglie `stoch`/`ctrl39`, la matrice completa, gli overlay e la deriva dell'attore.
- [ ] Valutare il recupero AB06 solo dopo la chiusura fail-closed della Fase 0; nessun DAgger o training prima del gate.
- [ ] Rendere osservabili separatamente nei futuri run RLlib: runner configurati/sani, batch corrente, contatore lifetime e generation/replacement.
- [ ] Chiarire la semantica `cycle_valid` dopo la cancellazione di un heel strike nella FSM v3.
- [ ] Generalizzazione multimodello con EPIC: progettare split per soggetto e superare un test su almeno un soggetto completamente held-out.
- [ ] TODO tecnici ereditati dal 21/08: swing timeout 1,3 s; driver di qualifica versionato; migrazione dichiarata del probe `train_ppo_mlp.py:1408`; verifica del commit FSM/corridoio.
- [ ] TODO amministrativi storici: pin V20, anomalia `best == last`, metrica `fit_p2` V12R3 e deflessione SEA iniziale.

