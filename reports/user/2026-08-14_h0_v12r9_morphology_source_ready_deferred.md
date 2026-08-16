# H0 V12R9 morphology causale: source-ready, esecuzione differita

## Esito

È stato creato il namespace additivo
`Trajectory Generator/baseline_MLP/validation/v12r9morph/` per qualificare il
Morphology Corridor sul candidato V12R9. Il protocollo è **source-ready**, ma
resta intenzionalmente non congelato e non eseguito finché i tre prerequisiti
V12R9 recovery, V12R9-Q3 e V12R9 checkpoint-zero non producono i rispettivi
terminal PASS sulla stessa identità candidato/checkpoint.

Non sono stati creati protocol freeze, execution lock, noise tape, claim,
rollout o output di training.

## Problema

Il protocollo morphology precedente era legato agli endpoint V12R8, una
lineage conclusa con terminal FAIL. Non era quindi lecito riusarne freeze,
lock o risultati. Serviva un port esclusivamente V12R9 che conservasse:

- detector binario V26 attivo;
- sei coppie fisiche control-then-positive, quindi 6 control + 6 positive;
- peso morphology `0.0` nel control e `0.0025` soltanto nel positive;
- delay causale stretto di 40 ms;
- drop fail-safe del terminal flush troppo giovane con motivo
  `episode_end_before_delay`;
- fail-closed permanente per emissioni anticipate non terminali;
- zero update actor/critic/PPO durante l'A/B;
- esecuzione one-shot, senza retry o resume;
- ripresa successiva dal checkpoint-zero completo, mai un nuovo warm-start
  actor-only.

## Soluzione

Il port usa soltanto gli endpoint pubblici V12R9:

- recovery V12R9, schema `1290`;
- qualifica Q3 V12R9, schema `1293`;
- checkpoint-zero V12R9, schema `1294`.

Il contratto morphology usa lo schema `1295` e rifiuta riferimenti residui a
namespace V12R8. La matrice Q3 è fissata ai casi indipendenti:

- `deterministic_offset_minus_0p30`;
- `deterministic_offset_plus_0p30`;
- `stochastic_nominal_seed_130`;
- `stochastic_nominal_seed_131`;
- `stochastic_nominal_seed_132`;
- `stochastic_nominal_seed_133`.

Questi casi sono verificati come disgiunti dalle condizioni observer e
development V12R9.

Il runtime additivo sostituisce il buffer morphology sia nel modulo corridor
sia nel wrapper reward, inclusi supervisor, child process e worker Ray. Il
file congelato `experimental_morphology_corridor.py` non è stato modificato ed
è ora legato esplicitamente all'hash:

`258700b5da99aa9110a92039834bf4061ca81b9eb7ea770b93919a0b3105e801`

## Working directory e handoff training

Freeze, execute, verifica terminale, costruzione handoff e launcher training
richiedono `cwd=repository_root` prima delle operazioni sensibili. Il requisito
è registrato in protocollo, lock, claim, receipt, ledger e handoff.

L'handoff pinna esplicitamente il launcher causale:

`Trajectory Generator/baseline_MLP/validation/v12r9morph/run_h0_v12r9_morphology_training.py`

Il comando finale è un resume dal checkpoint-zero completo per 50 nuovi
update. L'handoff pubblica inoltre l'endpoint del protocollo additivo
`v12r9training`, con:

- preflight restore obbligatorio su 12 EnvRunner prima del training;
- status richiesto
  `PASS_H0_V12R9_TRAINING_PREFLIGHT_12_RUNNER_RESTORE`;
- audit post-run obbligatorio dei 50 update;
- status richiesto
  `PASS_H0_V12R9_TRAINING_50_UPDATE_INTEGRITY`.

È stata inoltre aggiunta una checklist finale automatica e read-only. Prima
che gli artefatti richiesti esistano restituisce
`DEFERRED_H0_V12R9_MORPH_FINAL_ABI_HASH_AUDIT`, elenca i path mancanti e non
invoca alcun verificatore semantico. Quando la lineage sarà completa,
ricostruirà interamente in memoria upstream gate, source/input closure,
condizioni Q3, RLModule checkpoint-zero, protocollo, lock e piano delle 12
rollout, producendo gli hash prospettici senza effettuare write, freeze,
  rollout o update. La checklist consuma inoltre l'ABI pubblica
  `VALIDATION_ABI_LITERALS` e il `source_check()` del validator training,
  legando con hash sia validator sia contratto senza importare o aprire
  superfici live di preflight.

## Strategia di esecuzione congelabile

Quando i prerequisiti saranno terminal PASS sulla stessa lineage:

1. invocare i tre verificatori semantici nativi, senza fidarsi di flag copiati;
2. re-hashare sorgenti, profili, tape, candidato, checkpoint e RLModule
   ripristinato;
3. pubblicare protocol freeze ed execution lock con write no-clobber;
4. eseguire una sola volta le sei coppie, sempre control prima di positive;
5. richiedere identità byte/hash di osservazioni, azioni, dinamica, eventi,
   reward base, morphology loss e diagnostica causale tra i due bracci;
6. accettare come unica differenza di configurazione
   `morphology_weight` e `morphology_causal_allow_effects`;
7. pubblicare l'handoff soltanto se tutte le coppie e l'aggregate gate passano;
8. eseguire il preflight training esterno, quindi il resume da 50 update e
   infine l'audit d'integrità obbligatorio.

Qualunque errore dopo il claim produce terminal FAIL e vieta retry, sweep e
training positivo sulla stessa lineage.

## File aggiunti

- `Trajectory Generator/baseline_MLP/validation/v12r9morph/__init__.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9morph/h0_v12r9_morphology_contract.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9morph/h0_v12r9_morphology_gates.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9morph/h0_v12r9_morphology_causal_runtime.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9morph/h0_v12r9_morphology_physical_rollout.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9morph/freeze_h0_v12r9_morphology.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9morph/run_h0_v12r9_morphology.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9morph/run_h0_v12r9_morphology_training.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9morph/sitecustomize.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9morph/audit_h0_v12r9_morphology_readiness.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9morph/test_h0_v12r9_morphology_scaffold.py`

Non sono stati modificati il plugin C++, la dinamica SEA o la semantica del
comando degli attuatori.

## Test e verifiche

- `pytest validation/v12r9morph`: **28/28 PASS**, incluso un vero worker Ray;
- contract self-check: **10/10 PASS**;
- static readiness ABI audit: **12/12 PASS** su 11 sorgenti morphology;
- readiness live con artefatti ancora assenti: **DEFERRED**, 23 path mancanti,
  zero verificatori semantici invocati e zero write;
- training validation public source ABI: **9/9 PASS**;
  - validator SHA-256:
    `67c8f6b55dc7bd5d3ba964cba542159e315b0a35033bb73bc5aa29a914fa4837`;
  - contract SHA-256:
    `770380e27afc1f21a42207c86bb2485c4b54215704433a85a9781b0d5b66ea62`;
- suite indipendente `validation/v12r9training`: **29/29 PASS**;
- Ruff format: **11 file formattati/verificati**;
- Ruff check: **PASS**;
- compilazione Python dell'intero namespace: **PASS**;
- controllo hash del corridor congelato: **PASS**;
- controllo assenza riferimenti V12R8 nel namespace: **PASS**;
- controllo cwd errata da `baseline_MLP`: freeze, execute, handoff e launcher
  tutti respinti prima dell'I/O;
- controllo che il launcher finale sia sempre quello strict-delay V12R9morph:
  **PASS** su macOS arm64 e Windows x86_64;
- controllo assenza di artefatti non sorgente nel nuovo namespace: **PASS**.

## TODO

- Attendere terminal PASS V12R9 recovery, V12R9-Q3 e V12R9 checkpoint-zero
  sulla stessa identità candidato/checkpoint.
- Rieseguire l'audit ABI/hash del namespace morphology contro gli artefatti
  terminali definitivi.
- Solo allora pubblicare freeze e lock ed eseguire una volta le 12 rollout
  fisiche morphology.
- Dopo terminal PASS morphology, ottenere il preflight V12R9 training su 12
  EnvRunner, avviare il resume da 50 update e produrre l'audit post-run
  obbligatorio.
