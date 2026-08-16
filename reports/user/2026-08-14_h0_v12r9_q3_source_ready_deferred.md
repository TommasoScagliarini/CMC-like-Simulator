# H0 V12R9-Q3 — qualifica indipendente source-ready e deferred

Data: 2026-08-14

## Stato della milestone

Il successore additivo `V12R9-Q3` è implementato e testato come protocollo
source-only, ma resta correttamente **deferred** fino a un ledger terminale
`PASS` di V12R9. Non sono stati pubblicati protocol freeze, execution lock,
noise tape o rollout canonici Q3.

L'audit ABI/hash successivo all'handoff definitivo dei sorgenti V12R9 è
concluso: i sette hash di produzione R9 coincidono con lo snapshot consegnato,
la suite Q3 resta interamente verde e il binding ai cinque verifier pubblici R9
è invariato.

## Problema

V12R8 è terminato irreversibilmente in `FAIL` durante la prima collection e il
suo Q3 non può quindi essere riaperto, ritentato o retargettato in-place. Era
necessario creare un namespace nuovo che accettasse esclusivamente il candidato
completo prodotto da V12R9 e i cinque verifier ufficiali della sua lineage.

L'audit ha inoltre protetto due separazioni essenziali:

- un record candidato deve contenere l'intero albero a cinque file, inclusa la
  lista `files`; la proiezione a soli `path/tree_sha256/file_count` che causò il
  fallimento terminale R8 è rifiutata;
- la matrice Q3 deve restare realmente held-out rispetto alle sei condizioni
  observer/development R9. Q3 usa quindi `-0.30 s`, `+0.30 s` e seed
  `130-133`; i seed R9 `126-128` sono esclusi e rifiutati dai test.

## Soluzione e strategia

È stato creato il namespace:

`Trajectory Generator/baseline_MLP/validation/v12r9q3/`

Il protocollo conserva la semantica Q3 già validata:

- sei casi indipendenti, eseguiti prima tutti sul baseline e poi tutti sul
  candidate, per un totale di 12 rollout;
- stesso noise tape per ciascuna coppia condition-matched;
- detector binario V26 obbligatorio sul ramo candidate;
- morphology corridor causale ritardato costruito e diagnosticato, ma con
  peso, effetti causali e hard termination esattamente a zero;
- identità byte della reward a morphology zero e gate fisici/non-inferiority
  senza compensazione fra casi;
- zero actor, critic e PPO update durante Q3;
- nessun retry, resume, rescue, sweep, checkpoint-zero, morphology positiva o
  promozione runtime.

Il binding prerequisite è fail-closed su cinque endpoint V12R9:

1. `verify_protocol_freeze()`;
2. `verify_execution_lock(require_pristine=False)`;
3. `verify_candidate_freeze_receipt()`;
4. `verify_final_development_receipt()`;
5. `verify_terminal_ledger()`.

Oltre al risultato dei verifier, Q3 ricontrolla semanticamente candidate tree,
actor manifest `35 -> 512 -> 512 -> 2`, stage order terminale, contabilità
fisica R9, singolo fit R9, sei development puri e chiusura dei percorsi Q3.

## File introdotti

- `Trajectory Generator/baseline_MLP/validation/v12r9q3/__init__.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9q3/h0_v12r9_q3_artifacts.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9q3/h0_v12r9_q3_prerequisites.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9q3/h0_v12r9_q3_qualification_contract.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9q3/h0_v12r9_q3_qualification_gates.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9q3/test_h0_v12r9_q3_source_scaffold.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9q3/test_h0_v12r9_q3_real_tree_abi.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9q3/runtime/__init__.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9q3/runtime/freeze_h0_v12r9_q3_qualification_protocol.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9q3/runtime/prepare_h0_v12r9_q3_noise_tapes.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9q3/runtime/h0_v12r9_q3_physical_rollout.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9q3/runtime/run_h0_v12r9_q3_qualification.py`
- `Trajectory Generator/baseline_MLP/validation/v12r9q3/runtime/test_h0_v12r9_q3_runtime.py`

Nessun file V12R8, V12R8-Q3 o V12R9 è stato modificato da questo port.

## Test e verifiche

- suite V12R9-Q3 completa: `57 passed`;
- contract self-check: `12/12` check PASS;
- source closure corrente: `PASS_H0_V12R9_Q3_SOURCE_CLOSURE`, 43 record;
- audit indipendente dello snapshot R9 finale: `76 passed`, `ruff check` PASS,
  `ruff format --check` PASS su 14 file e locked-input attestation PASS;
- hash finali dei sette sorgenti di produzione R9 verificati uguali allo
  snapshot di handoff; ABI pubblica dei cinque verifier invariata;
- durante l'audit R9 sono stati rimossi due wrapper pubblici R7 incompatibili
  con l'autorità del successore ed è stato corretto il binding schema delle
  quattro nuove label R9, con regressioni positive e negative dedicate;
- albero R6 reale riletto dai byte: cinque file e tree SHA-256
  `340c2c65c2300a90ce46c09837e679a99e5dea09ce3935574ef5345fafb709f3`;
- regressione full-tree: la proiezione priva di `files` è rifiutata;
- test parametrico: `stochastic_nominal_seed_126`, `127` e `128` sono rifiutati
  dal contratto Q3;
- generazione noise in memoria (`--build-only`): PASS, senza materializzazione;
- `ruff check`: PASS;
- `ruff format --check`: PASS su 13 file;
- `compileall`: PASS;
- verifica namespace: nessun JSON/NPZ Q3 canonico creato;
- hash dei dieci sorgenti V12R8-Q3 ricontrollati invariati rispetto allo
  snapshot iniziale.

## Comandi della futura fase canonica

Questi comandi non sono stati eseguiti e restano subordinati al terminale
`PASS` V12R9 e a un audit finale dell'ABI:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r9q3/runtime/freeze_h0_v12r9_q3_qualification_protocol.py' --live-preflight
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r9q3/runtime/freeze_h0_v12r9_q3_qualification_protocol.py' --freeze
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r9q3/runtime/prepare_h0_v12r9_q3_noise_tapes.py' --prepare
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r9q3/runtime/run_h0_v12r9_q3_qualification.py' --live-preflight
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r9q3/runtime/run_h0_v12r9_q3_qualification.py' --execute
/opt/anaconda3/envs/envCMC-rllib/bin/python 'Trajectory Generator/baseline_MLP/validation/v12r9q3/runtime/run_h0_v12r9_q3_qualification.py' --verify
```

## TODO

- mantenere Q3 chiuso se V12R9 non termina con ledger `PASS` e candidato
  completo a cinque file;
- soltanto dopo Q3 terminale `PASS`, passare al protocollo separato
  checkpoint-zero/warm-start;
- mantenere la morphology positiva fuori da Q3 e validarla nel protocollo
  causale dedicato.
