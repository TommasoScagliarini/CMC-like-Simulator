# Screening RIGHT39 prima della continuazione alla 60

Il 23 settembre 2026 l'utente ha autorizzato cinque seed di rumore per ciascuno
dei checkpoint RIGHT39 best35 e last40, seguiti dalla continuazione del
checkpoint completo40 fino all'iterazione **60 assoluta**. Non sono autorizzate
da questo piano continuazioni automatiche alla80/100 o l'oscuramento.

## Protocollo

- Dieci nuovi rollout nativi, con rumore originale (`noise_scale=1`), seed
  gaussiani123–127, seed ambiente123, start12,99s e limite500 passi.
- Massimo quattro simulazioni contemporanee, ciascuna con un thread numerico.
  Il training viene avviato separatamente, quando lo screening è terminato.
- Moduli immutabili `milestone_iteration_000035/rl_module_last` e
  `milestone_iteration_000040/rl_module_last` del run
  `MLP39_v26_aligned_right_phase_fsmv3_40iter_2026-09-23`.
- Riferimento destro V26, FSMv3, clock e prescribed visibili100%; nessuna
  modifica a produzione, checkpoint o impostazioni della policy.
- `run_suite.py` registra stato, comandi, log, esiti e62 hash degli input in
  `../../runs/plans/right_reference_screening_2026-09-23/`.
- `analyze.py` verifica rumore, azioni, reward, riferimento e clock; aggiunge
  il confronto con cinque rollout archiviati V26 best87 e cinque LEFTbest72.
  L'opzione `--partial` non certifica lo screening completo.

## Interpretazione

Il confronto V26 usa la policy storica valutata nel contenitore attuale con
FSMv3; non cinque rollout stocastici registrati ad agosto. Gli stessi campioni
gaussiani standard sono verificati per seed, ma ciascuna policy mantiene la
propria deviazione standard appresa: il rumore assoluto può quindi differire.

Si distinguono arrivo a500 passi, cicli FSM validi, eventi invalidi e
penetrazione. La condizione «episodio completo con almeno due cicli e zero
eventi invalidi» è una metrica descrittiva aggiuntiva, non una nuova definizione
di completamento né una soglia automatica di approvazione.

Le metriche fisiche comuni usano il prefisso condiviso da tutti gli episodi;
le eventuali righe terminali senza un nuovo stato fisico ne sono escluse.
I contatori di episodio restano separati. I cinque seed con start fisso sono
uno screening, non una dimostrazione di equivalenza statistica.

## Artefatti finali

Quando tutti i casi sono terminati, `metrics.json` contiene gli esiti e le
verifiche; `input_hashes.json` e `analysis.log` ne documentano le sorgenti.
La figura comparativa viene salvata nella root del repository, in
`plot/09_23_2026_right_screening_comparison/paired_penetration.png`.

La continuazione separata è descritta in
`../right_reference_resume60_2026-09-23/README.md`. Questo screening non avvia
autonomamente il training: il processo principale esamina gli esiti prima
di lanciare il monitor della continuazione.
