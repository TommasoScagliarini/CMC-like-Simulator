# H0 V12R8 morphology: runtime latent-live e training gate

Data: 2026-08-14

## Esito della milestone

Il precedente scaffold `v12r8morph`, deliberatamente non eseguibile, è stato
convertito in un protocollo **latent-live fail-closed**. Il codice è pronto a:

1. invocare i verifier semantici nativi di R8, R8-Q3 e checkpoint-zero;
2. congelare protocollo e lock soltanto se i tre terminali sono PASS per lo
   stesso candidato, actor digest e checkpoint completo;
3. eseguire sei coppie fisiche nell'ordine `control -> positive`, quindi 12
   rollout locali da 500 step;
4. autorizzare e pubblicare il comando di resume da checkpoint-zero soltanto
   dopo il PASS di tutte le evidenze morphology.

Non sono stati eseguiti il freeze canonico morphology né rollout fisici. Al
momento della chiusura di questa milestone, il verifier nativo R8 restituisce
`FAIL_H0_V12R8_RECOVERY_PIPELINE_TERMINAL` (`passed=false`, candidato assente),
mentre i terminali Q3 e zero non esistono ancora. Il blocco è quindi upstream e
il nuovo freezer morphology lo rifiuta correttamente.

## Problemi risolti

Lo scaffold precedente presentava i seguenti limiti:

- `freeze()` ed `execute()` erano intenzionalmente inerti;
- i terminali upstream potevano essere descritti ma non verificati mediante i
  verifier nativi;
- le trace Q3 esistenti non contengono tutte le superfici necessarie per un A/B
  causale (osservazioni complete, dinamica, eventi grezzi, reward base,
  morphology loss e campioni causali);
- reward e causal ledger non erano completamente hash-bound e verificati per
  singolo step/campione;
- una morphology loss tutta zero avrebbe potuto simulare una qualifica senza
  dimostrare alcun effetto reale;
- mancavano claim one-shot, ledger terminale FAIL/PASS e handoff di training
  autorizzato esclusivamente dal PASS.

È inoltre emerso un problema nel sorgente morphology congelato da R8: al flush
terminale il buffer può restituire campioni più giovani del delay causale di
40 ms. Quel file fa parte degli 82 sorgenti congelati R8 e non è stato
modificato.

## Soluzione e strategia

### ABI upstream e fallback fisico obbligatorio

Il contratto usa gli endpoint reali e stabili:

- R8: `verify_terminal_ledger`;
- R8-Q3: `verify_terminal_ledger`;
- R8-zero: `verify_terminal_pass`.

Ogni risultato è legato all'artefatto verificato e all'hash canonico del valore
restituito. Devono coincidere candidato a cinque file, actor digest, albero
completo del checkpoint-zero e config risolta.

Le trace candidate Q3 correnti vengono ispezionate direttamente. Poiché
mancano superfici causali complete, il solo percorso autorizzato è
`paired_rerun`: sei control fisici più sei positive fisici, con stesso modulo,
stessa condizione e stesso noise tape.

### A/B morphology

Le due configurazioni derivano direttamente dal contratto checkpoint-zero e
differiscono in esattamente due campi:

- `morphology_weight`: `0.0 -> 0.0025`;
- `morphology_causal_allow_effects`: `0.0 -> 1.0`.

Restano fissi detector binario V26, debounce, event contract, delay `0.04 s`,
latenza massima `0.01 s` e hard termination morphology disabilitata.

Ogni arm salva e ribinda:

- trace completa;
- reward ledger;
- causal ledger;
- noise tape;
- RLModule già esportato dopo il restore del checkpoint-zero;
- stream canonici di osservazioni, azioni, dinamica, eventi, reward senza
  morphology e morphology loss.

Il gate di coppia richiede identità hash delle superfici non-reward,
ricomposizione IEEE esatta del reward, ordine control prima di positive,
validità causale per ogni campione e zero update. L'aggregate PASS richiede
almeno un campione con penalità morphology strettamente positiva: il caso
all-zero viene respinto.

### Correzione additiva del flush sotto 40 ms

È stata aggiunta una implementazione versionata, senza cambiare alcun sorgente
congelato R8. La classe
`StrictDelayCausalMorphologyBuffer` viene iniettata nei simboli realmente usati
dal reward:

- `experimental_morphology_corridor.CausalDelayedMorphologyBuffer`;
- `reward_function.CausalDelayedMorphologyBuffer`.

I campioni terminali con età inferiore a 40 ms vengono esclusi dal reward e
registrati come drop atteso `episode_end_before_delay`. Un'eventuale emissione
anticipata non terminale rende il buffer permanentemente fail-closed. Il valore
`_last_emitted_sample_time_s` viene ricalcolato soltanto sui campioni trattenuti.

Il launcher finale imposta un marker opt-in e un `sitecustomize.py` versionato,
così la stessa iniezione è attiva nel supervisore, nel child process di
`train_ppo_mlp.py` e nei worker Ray. Il comando finale punta al launcher, usa
`--resume-from` sul checkpoint-zero e vieta `--warm-start` e
`--warm-start-raw`.

## File della milestone

- `Trajectory Generator/baseline_MLP/validation/v12r8morph/__init__.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8morph/h0_v12r8_morphology_contract.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8morph/h0_v12r8_morphology_gates.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8morph/freeze_h0_v12r8_morphology.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8morph/run_h0_v12r8_morphology.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8morph/h0_v12r8_morphology_physical_rollout.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8morph/h0_v12r8_morphology_causal_runtime.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8morph/run_h0_v12r8_morphology_training.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8morph/sitecustomize.py`
- `Trajectory Generator/baseline_MLP/validation/v12r8morph/test_h0_v12r8_morphology_scaffold.py`

Nessuno degli 82 source path congelati da R8 è stato modificato. Il verifier
del protocol freeze R8 continua a restituire
`PASS_H0_V12R8_RECOVERY_PROTOCOL_FREEZE` con `source_count=82`.

## Test e verifiche

- Suite completa con interprete RLlib, fuori sandbox per consentire il worker
  Ray reale: **21 passed**.
- Flush terminale reale/sintetico sotto 40 ms: campione escluso, contatori
  corretti, nessun reward anticipato.
- Violazione anticipata non terminale: fail-closed persistente verificato su
  chiamate successive.
- Child interpreter Python reale: `sitecustomize` installa il runtime atteso.
- Worker Ray reale: runtime ID corretto nel processo remoto.
- Mutazione dell'attestazione d'iniezione: arm gate FAIL.
- Mutazioni causali: delay 39 ms, latenza 11 ms, diagnostica fail-closed e delay
  dichiarato errato vengono respinti.
- Mutazioni di stream/config/reward: pair gate FAIL.
- Aggregate morphology all-zero: FAIL.
- `ruff check`: PASS.
- `py_compile` su tutti i moduli `v12r8morph`: PASS.
- `verify_protocol_freeze()` R8: PASS su 82 sorgenti.
- `verify_terminal_ledger()` R8: terminal FAIL upstream, correttamente non
  accettato dal morphology freezer.

## Hash SHA-256 dei sorgenti validati

| File | SHA-256 |
|---|---|
| `__init__.py` | `a7dc640a07487df46c082499a052669fbaf7bb98dfc4ac2576e29b6c75eacbff` |
| `freeze_h0_v12r8_morphology.py` | `c86795cb7911ba740e0f1fe26284dedb0f1c79924c18d3c16662ee48a0da7967` |
| `h0_v12r8_morphology_causal_runtime.py` | `48aae381012a172d919ab752da39338a89870d83124f80cd6c6ec685bce097ee` |
| `h0_v12r8_morphology_contract.py` | `19282a07b58965e09b058966364d2f27e5d25b2cad8cac9bb516ff17d06293c2` |
| `h0_v12r8_morphology_gates.py` | `6fffff2ae87d9ed42cb3486eb0a65beee9a0bbc21f2f5adf613a3a27a7339f96` |
| `h0_v12r8_morphology_physical_rollout.py` | `1133c227662d916dd98a1e29d935d6753e5790ff9b21e647050d32f3b8e99aee` |
| `run_h0_v12r8_morphology.py` | `18f901fe9fec1b57a929f290cbed43da8a1e951fa31d721994e8a4413bb6e100` |
| `run_h0_v12r8_morphology_training.py` | `06c75903c00b3736e0074a28a2b7978f5301df19460b5481c6d6df7146798916` |
| `sitecustomize.py` | `90f01841ddfff6c3285c4ea6b581a24cecf349345701efdcddd5557865b78fff` |
| `test_h0_v12r8_morphology_scaffold.py` | `fb2331a29af158a00c9b06c38f7ce9387dba7df2987e63f588181a830337065e` |

## Stato e TODO propagato

Stato: **source-ready come implementazione di riferimento, ma la lineage R8 è
chiusa terminalmente e non diventerà execution-ready**. Non verrà modificata o
rieseguita per ottenere artificialmente un PASS.

TODO:

1. completare l'additive successor R9 e ottenerne il terminal PASS senza
   modificare o rieseguire R8;
2. portare Q3 su R9 ed eseguirlo sullo stesso candidato;
3. portare checkpoint-zero su R9/Q3 e verificarne il terminal PASS;
4. portare questa implementazione in un namespace morphology R9, collegando i
   tre nuovi endpoint semantici;
5. solo allora pubblicare freeze/lock ed eseguire una sola volta le 12 rollout
   fisiche;
6. se e solo se il terminale morphology R9 è PASS, usare l'handoff pubblicato
   per il resume di 50 update dal checkpoint-zero.
