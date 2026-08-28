# TODO — Generalizzazione multi-modello della policy con dataset EPIC

**Data:** 22 agosto 2026  
**Stato:** differito; da propagare nei daily report fino a completamento  
**Priorità corrente:** successiva al recupero e alla qualifica della qualità AB06

## Contesto e decisione

Questo TODO formalizza la conversazione del 22 agosto 2026 nella quale è stata discussa la dipendenza dal modello del processo Teacher–Student/DAgger. Il metodo non è intrinsecamente specifico di AB06, ma la pipeline corrente lo è statisticamente: teacher, IK, GRF, dinamica di raccolta, action bounds e Morphology Corridor derivano da AB06.

L'actor 35D è causalmente deployable e può essere tecnicamente caricato dove observation e action schema coincidono; questo non dimostra generalizzazione cross-subject o cross-model. La decisione approvata è recuperare prima la qualità AB06, mantenendo la generalizzazione come workstream separato.

## Obiettivo futuro

Costruire e qualificare una policy capace di operare su modelli/soggetti diversi da AB06, usando EPIC come dataset primario e separando formalmente:

1. portabilità del software e dello schema;
2. trasferimento zero-shot;
3. adattamento few-shot specifico del soggetto;
4. generalizzazione scientifica su un soggetto/modello mai visto.

## Strategia prevista

- Inventariare i soggetti e trial EPIC, con task, velocità, IK, ID, GRF, metadata antropometrici, modello OpenSim e compatibilità delle coordinate protesiche.
- Definire uno schema canonico/adimensionale per osservazioni e azioni, includendo le trasformazioni fra coordinate del modello e coordinate della policy.
- Costruire teacher specifici per soggetto/modello e raccogliere DAgger su una distribuzione multi-modello, non solo su perturbazioni dello stesso AB06.
- Valutare due architetture:
  - actor universale condizionato da massa, lunghezze segmentali, inerzie, velocità/task e parametri SEA;
  - policy di forma cinematica normalizzata più adattatore/decoder specifico del modello.
- Sostituire il corridoio AB06 con un profilo multi-soggetto normalizzato oppure con profili soggetto-specifici selezionabili.
- Riservare almeno un intero soggetto/modello EPIC come leave-one-model-out sigillato.
- Preregistrare gate distinti per zero-shot e few-shot, includendo cinematica phase-aligned, coppie, cicli, reserve, sicurezza e contatori FSM.

## Criterio di completamento

Il TODO potrà essere chiuso soltanto dopo un PASS su almeno un soggetto/modello EPIC completamente escluso da teacher, DAgger, PPO, scelta degli iperparametri e costruzione del corridoio. Più trial dello stesso soggetto non costituiscono una prova cross-subject.

## Collegamenti

- Piano AB06 approvato: `reports/plans/2026-08-22_piano_operativo_recupero_pipeline_exnovo_v26_fsmv3_morphology.md`.
- Dataset e modelli EPIC presenti nel repository: `models/AB06-raw/`, `models/AB07-raw/`, `models/AB08-raw/`, `tools/build_abx_seasea_pipeline.py` e relativi protocolli/artefatti di conversione e validazione.

## Modifiche e verifiche correnti

In questa registrazione non sono stati modificati codice, policy, dataset o configurazioni di training. È stato soltanto formalizzato il TODO e separato il suo scope dal recupero AB06 in corso.
