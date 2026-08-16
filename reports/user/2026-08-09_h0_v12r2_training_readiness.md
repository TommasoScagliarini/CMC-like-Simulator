# H0 V12R2 — protocollo training/execution-ready su macOS arm64

Data: 2026-08-09

## Esito

La lineage additiva V12R2 è stata implementata, verificata e sigillata con il
seguente contratto:

```text
AB06_H0_PRIMARY_SPLIT_V12R2_V26_AUTONOMY_RECOVERY
H0_V12R2_V26_RECOVERY_WEIGHTED_PURE_PROBE_DAGGER
schema_version = 122
```

L'execution lock canonico dichiara:

```text
PASS_H0_PRIMARY_SPLIT_V12R2_EXECUTION_LOCK
training_ready = true
next_stage = EXECUTE_V12R2_ONCE
```

Questa readiness ha uno scope preciso: protocollo, runtime e macchina sono
pronti per avviare una sola volta la pipeline V12R2 con
`/opt/anaconda3/envs/envCMC-rllib/bin/python3.10` su Darwin arm64. Il run root,
il pipeline claim e il ledger V12R2 sono ancora assenti; `--execute-once` non è
stato invocato.

Questo stato **non equivale ancora a `H0_TRAINING_READY` scientifico**. Non
esiste ancora un candidato V12R2 persistito, i rollout development e la
qualification non sono stati eseguiti e non è stato pubblicato alcun
`checkpoint_zero` o comando di warm-start PPO.

Windows x86_64 resta operativo in modalità NO-GO fail-closed perché nel
repository corrente mancano i due binari esatti:

- `plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll`;
- `plugins/OnlineGRFContact.dll`.

Non è stato rinominato o accettato come sostituto alcun DLL storico.

## Problema affrontato

Il tentativo V12R1 era terminato in modo definitivo al primo stage `fit_p0`:

```text
FAIL_H0_PRIMARY_SPLIT_V12R1_PIPELINE_TERMINAL
KeyError: 'completed_v12r1_collection_rounds'
```

Il problema non era scientifico né legato a OpenSim. Era un errore strutturale
nel meccanismo usato per derivare V12R1 dal contratto V12. Il rewriter dei
literal stringa modificava le stringhe semplici contenute nei code object, ma
non applicava la stessa trasformazione alle stringhe annidate in tuple o
`frozenset`.

Di conseguenza:

- il lookup era diventato `completed_v12r1_collection_rounds`;
- il producer e il keyset atteso conservavano
  `completed_v12_collection_rounds`;
- esisteva la stessa incompatibilità latente per
  `v12_dagger_sample_count`/`v12r1_dagger_sample_count`.

V12R1 aveva aperto un solo claim e tentato un solo fit/update actor, senza
confermarlo. Il ledger registra zero stage completati, zero reset, zero step e
zero label offline. Retry, sweep e rescue erano vietati; V12R1 non è stato
ritentato o riscritto.

## Soluzione V12R2

V12R2 è stata creata in una directory isolata e additiva:

```text
Trajectory Generator/baseline_MLP/validation/v12r2/
```

La correzione principale consiste nell'eliminare completamente la riscrittura
dei literal del bytecode. Le funzioni ereditate usano gli stessi code object
V12 originali e il wire format conserva intenzionalmente le chiavi legacy
stabili:

```text
completed_v12_collection_rounds
v12_dagger_sample_count
```

Sono stati inoltre introdotti:

- un self-check totale del contratto di fit per P0-P3;
- un unico producer canonico dell'audit del corpus nel fitter;
- gate che restituiscono FAIL controllato sui payload malformati senza
  sollevare `KeyError`;
- scoperta robusta della root del repository per fitter e labeler;
- binding byte-exact dei 12 artifact terminali V12R1;
- freeze e verifier phase-aware, capaci di ricostruire lo snapshot originale
  anche dopo design audit ed execution lock;
- pubblicazioni no-clobber con creazione esclusiva;
- readiness runtime fail-closed per sistema, architettura, interprete e plugin;
- `.gitattributes` locale a V12R2, senza modificare quello già congelato in
  V12R1.

V12, V12R1, il relativo run terminale e tutti gli artifact storici sono
rimasti immutati.

## Strategia scientifica congelata

La pipeline V12R2 mantiene l'obiettivo V12 di colmare il gap tra raccolta
teacher-shielded e comportamento autonomo:

- ogni fit P0-P3 riparte dall'H0 sorgente congelato;
- il seed corpus è il corpus V11 P3 da 6.000 righe, byte-bound;
- dopo ogni fit è previsto un pure probe senza teacher, blending o safety
  latch;
- il labeler osservatore opera soltanto dopo la chiusura verificata del probe;
- i label dei pure probe sono cumulativi e associati allo stesso stato
  osservato;
- le righe recovery ricevono peso crescente da 1 a 100 in funzione della
  penetrazione precedente;
- le righe pure-probe fuori copertura rispetto al riferimento V11 ricevono
  peso 100;
- la massa grezza di ogni episodio aggiunto viene normalizzata a 500;
- reset e riga terminale sono conservati, incluso il prefisso di un probe
  fallito;
- critic e PPO restano fuori scope.

Il piano congelato contiene 26 stage e 250 percorsi di mutazione dichiarati.
I percorsi massimi sono 136 caratteri per l'output finale e 150 per il nome
temporaneo esclusivo, entro il limite relativo multipiattaforma di 160.

Il contratto del corpus verificato è:

| Stage | Campioni | Seed V11 | DAgger V12 | Label pure probe | Reset | Round completati |
|---|---:|---:|---:|---:|---:|---|
| P0 | 6.000 | 6.000 | 0 | 0 | 12 | nessuno |
| P1 | 7.001 | 6.000 | 1.000 | 1 | 15 | 1 |
| P2 | 8.002 | 6.000 | 2.000 | 2 | 18 | 1, 2 |
| P3 | 9.003 | 6.000 | 3.000 | 3 | 21 | 1, 2, 3 |

I payload completi P0 e P3 sono stati provati contro il vero `fit_gate`. Tutti
i quattro stage superano il self-check; payload incompleti o con chiavi
V12R1/V12R2 spurie falliscono senza eccezioni.

## Design audit unico

Il design audit canonico ha eseguito una sola riproduzione P0 in memoria:

| Metrica | Osservata | Gate | Esito |
|---|---:|---:|---|
| RMSE | `0,0049784962` | `<= 0,006` | PASS |
| Errore assoluto massimo | `0,0575412363` | `<= 0,060` | PASS |
| Errore massimo reset | `0,0008472800` | `<= 0,003` | PASS |
| Errore massimo fold normalizzazione | `4,1723e-7` | `<= 1e-6` | PASS |

Contabilità del design audit:

- fit actor eseguiti: 1;
- update actor: 1;
- update critic: 0;
- update PPO: 0;
- reset ambiente: 0;
- step ambiente: 0;
- label teacher offline: 0;
- checkpoint candidato persistiti: 0;
- retry, sweep e rescue: disabilitati.

Il fit usa l'actor standard `35 -> 256 -> 256 -> 2`, AdamW full-batch per 3.000
epoche seguito da LBFGS, `logstd` congelata, clock 0/1 bit-zero e
normalizzazione incorporata nel primo layer. Il design audit ha verificato
anche critic assente, parametri non actor byte-exact e output finiti.

## Freeze, audit ed execution lock

Gli artifact canonici pubblicati sono:

| Artifact | Stato | Byte | SHA-256 |
|---|---|---:|---|
| Protocol freeze | PASS | 248.719 | `014b72083b9f7ca469d659c1ef9f0bc10c0a6ba196f662b490fed1ea724fe34c` |
| Design audit | PASS | 31.792 | `af9c356ef34ebfa9523ea76d6e55537ff404b8221aeaf279c4287d27c6853bd0` |
| Execution lock | PASS | 89.368 | `c4497c73ce0d0f0a67758929ab4fd0654d10db71c485e6c60fa35bb3dc2bf101` |

Il lock lega esattamente:

- freeze e design audit V12R2;
- sette sorgenti operative future;
- i 12 artifact della lineage terminale V12R1;
- H0 sorgente e teacher evidence;
- 66 sorgenti e 54 input dell'evidenza runtime V11 ereditata;
- interprete, librerie e plugin della macchina corrente;
- ordine dei 26 stage e manifest dei 250 path.

Runtime sigillato:

| Componente | Valore |
|---|---|
| Sistema | Darwin arm64 |
| Python | 3.10.20 |
| NumPy | 2.2.6 |
| SciPy | 1.15.2 |
| Torch | 2.10.0 |
| Ray | 2.55.1 |
| SEA plugin | `plugins/libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib` |
| OnlineGRF plugin | `plugins/libOnlineGRFContact.dylib` |

I tre verifier ricostruiscono freeze, audit e lock nello stato post-lock con
esito PASS. I JSON sono strict e canonicali; i path sono regular file senza
symlink.

## Stato della pipeline

La pipeline scientifica V12R2 non è stata eseguita. Restano assenti:

```text
Trajectory Generator/baseline_MLP/validation/v12r2/h0_v12r2_run_20260809/
pipeline_claim.json
pipeline_ledger.json
```

L'assenza è intenzionale: la richiesta era arrivare allo stato training-ready,
non consumare il claim one-shot. L'execution lock autorizza come prossimo e
unico passaggio:

```bash
cd '/Users/tommy/Documents/CMC-like-Simulator - Claude'
env PYTHONDONTWRITEBYTECODE=1 \
  PYTHONPATH="$PWD:$PWD/validation:$PWD/Trajectory Generator:$PWD/Trajectory Generator/baseline_MLP:$PWD/Trajectory Generator/baseline_MLP/validation:$PWD/Trajectory Generator/baseline_MLP/validation/v12r2" \
  /opt/anaconda3/envs/envCMC-rllib/bin/python \
  'Trajectory Generator/baseline_MLP/validation/v12r2/run_h0_primary_split_v12r2_autonomy_recovery.py' \
  --execute-once
```

Un failure durante l'esecuzione deve essere considerato terminale: il
protocollo non autorizza retry, sweep o rescue.

## File introdotti

Contratto e runtime:

- `Trajectory Generator/baseline_MLP/validation/v12r2/.gitattributes`;
- `h0_primary_split_v12r2_autonomy_recovery_contract.py`;
- `h0_primary_split_v12r2_recovery_weighted_fitter.py`;
- `h0_primary_split_v12r2_pure_probe_observer_labeler.py`;
- `freeze_h0_primary_split_v12r2_autonomy_recovery.py`;
- `run_h0_primary_split_v12r2_design_audit.py`;
- `run_h0_primary_split_v12r2_autonomy_recovery.py`.

Test:

- `test_h0_primary_split_v12r2_autonomy_recovery_contract.py`;
- `test_h0_primary_split_v12r2_recovery_weighted_fit.py`;
- `test_h0_primary_split_v12r2_pure_probe_observer_labeler.py`;
- `test_freeze_h0_primary_split_v12r2_autonomy_recovery.py`;
- `test_h0_primary_split_v12r2_execution.py`.

Artifact:

- `h0_primary_split_v12r2_autonomy_recovery_protocol_freeze.json`;
- `h0_primary_split_v12r2_autonomy_recovery_design_audit.json`;
- `h0_primary_split_v12r2_autonomy_recovery_execution_lock.json`.

I nomi senza prefisso completo sono relativi a
`Trajectory Generator/baseline_MLP/validation/v12r2/`.

Non sono stati modificati plugin C++, semantica SEA, GRF primaria, detector o
FSM V26, checkpoint H0/V11, artifact V12/V12R1 o dati AB06 protetti.

Al momento della chiusura la directory `validation/v12r2/` e questo report
risultano untracked in Git. Gli artifact locali sono comunque byte-bound dal
freeze e dal lock; non è stato eseguito alcun commit o push.

## Test e verifiche

Verifiche finali:

- suite V12R2 pre-lock: 62/62 PASS;
- suite V12R2 post-lock: 62/62 PASS;
- Ruff: PASS;
- Black `--check`: PASS su 11 file Python;
- compilazione Python: PASS con cache esterna temporanea rimossa;
- freeze read-only: 22/22 invarianti PASS;
- verifica byte-exact V12R1: 12/12 artifact PASS;
- evidenza runtime V11: 66/66 sorgenti e 54/54 input PASS;
- protocol freeze `--verify`: PASS prima e dopo il lock;
- design audit `--verify`: PASS senza rieseguire il fitter;
- execution lock `--verify-execution-lock`: PASS;
- tre audit indipendenti read-only: GO per contratto, freeze e runner.

È stato inoltre eseguito uno smoke test reale dell'ambiente OpenSim con i due
plugin macOS: caricamento modello, un reset e uno step ad azione zero hanno
chiuso PASS con output diagnostico
`PASS_ENV_SMOKE (84,) (2,) 5 False False`. Questo smoke non ha scritto nel run
root V12R2. Un primo wrapper diagnostico aveva usato la chiave locale `seed`
anziché `runtime_seed` ed era terminato prima del reset; corretto il wrapper,
lo smoke reale è passato. Nessun sorgente di produzione è stato cambiato per
questo errore del solo harness diagnostico.

Il controllo simulato Windows conferma il comportamento fail-closed in assenza
dei due DLL esatti.

## TODO completati o superseduti

- [x] Autorizzare e creare una nuova lineage V12 separata, senza riaprire V11 o
  V12R1.
- [x] Inserire nel contratto un pure probe unblended dopo ogni candidato
  P0-P3.
- [x] Rendere rischio recovery, frequenza/durata del latch e copertura dei pure
  probe parte esplicita del corpus/gate.
- [x] Correggere il test one-shot rendendo i verifier phase-aware nello stato
  freeze, audit e lock.
- [x] Conservare V11, V12 e V12R1 byte-exact e non allentare il limite di
  penetrazione da 25 mm.
- [x] Mantenere invariati V26, GRF primaria e semantica SEA.

Questi punti sono completati a livello di design e infrastruttura. La loro
efficacia scientifica resta da dimostrare nell'esecuzione V12R2.

## TODO aperti e propagati

### Esecuzione e qualification

- [ ] Eseguire V12R2 una sola volta con il lock corrente; non ritentare in caso
  di failure terminale.
- [ ] Ottenere tutti i fit, pure probe, label, round di raccolta e sei rollout
  autonomi development PASS con almeno due cicli, zero safety stop e
  non-regressione SEA/reserve.
- [ ] Solo dopo un PASS development, eseguire V10Q/qualification indipendente.
- [ ] Aprire trial 05 e, soltanto dopo il relativo PASS, trial 06; mantenere
  03/07 chiusi come reserve.
- [ ] Eseguire zero-update save/reload, produrre un vero `checkpoint_zero` e
  pubblicare il comando warm-start soltanto dopo la qualification.
- [ ] Mantenere morphology reward positivo, corridor training e PPO chiusi
  finché non esiste `H0_TRAINING_READY` scientifico.

### Multipiattaforma e deployment

- [ ] Compilare e testare su Windows x86_64 i DLL esatti
  `SEA_Plugin_BlackBox_mCMC_impedence_ff.dll` e `OnlineGRFContact.dll`, quindi
  produrre un claim numerico separato per quella piattaforma.
- [ ] Validare i candidati futuri su multistart, seed held-out, recovery,
  soggetti/profili esterni, rumore e delay.
- [ ] Completare equivalenza host-target, latenza worst-case, HIL e contratto
  hardware prima di qualsiasi claim di deployment.
- [ ] Conservare come TODO storico l'inizializzazione della deflessione SEA
  coerente con la coppia richiesta.
