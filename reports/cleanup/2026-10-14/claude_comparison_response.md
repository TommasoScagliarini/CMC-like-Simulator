# Riconciliazione finale Claude ↔ Codex

In questa risposta non ho fatto nuove ispezioni: le misure live citate sono di Codex.

## 1. Snapshot e prima operazione

**Conclusione:** lo snapshot è completo per i file tracciati, ma esiste solo in locale. Secondo la policy documentata di GitHub non si può pushare così com'è, e non contiene gli ignorati.

- **Tracciati.** `dba4238` (tree `85c00158`) contiene i 65.125 path. Le 31 cancellazioni SNN erano già presenti e si recuperano da HEAD; Codex ne ha già estratto il README.
- **Remoto.** Contiene solo HEAD/main → `5782540`. Correggo la mia affermazione «non protetto fuori da questo disco»: altri backup non sono né noti né esclusi.
- **Push.** Codex ha misurato ciò che il mio piano rimandava. Tre path superano 100 MiB senza LFS:
  - `episodes_pre_gae.pkl`, 2 copie, 271.503.010 B;
  - `exploration_records.jsonl`, 180.033.948 B.

  Un push normale verrebbe quindi bloccato (non è stato provato). Per usare il remoto bisogna riscrivere `dba4238`, togliendo i tre file o migrandoli a LFS. Si può fare perché il commit non è pubblicato: la mia riserva sulla riscrittura riguardava solo la history pubblicata. Va fatto però dopo averne una copia integrale. La dimensione del pack `origin/main..HEAD` resta da misurare.
- **Ignorati** (88.351 file, 31,12 GB): sono fuori dal commit, salvo copie parziali (per esempio nel bundle V26C).

**Prima operazione e completamento (nulla è stato eseguito):**
1. `git bundle create --all` su un supporto esterno, poi `verify` e un clone di prova confrontando HEAD e tree.
2. Archivio esterno degli ignorati essenziali con manifest SHA-256: best47 e parent, plugin, `models/**/data`, `*-raw`, `results`, `plot` e log pinnati, `fine_results.jsonl`.
3. Restore in una directory separata, confronto con i pin e reload di best47 dalla copia ripristinata (aggiunta di Codex).

Il tag locale è un'ancora, non una protezione.

## 2. KEEP, rimozione, archivio, refactor

| Classe | Contenuto | Condizioni |
|---|---|---|
| KEEP | 16 `.py` root, 7 `.py` TG, plotter, `validate_sim_results.py`, `baseline_MLP` V26 con i moduli piatti legacy, config e profili, asset runtime in `validation/` (profilo V25, warm-start H0), parent June23 nel default di `warm_start.py`, plugin pinnati, sorgenti C++/CMake, pipeline ABx, modelli, `models/SEASEA/data`, best47 e parent, V26C, campagne di ottobre, report, sorgenti del paper | nessun rename |
| Rimozione tecnica | 2.159 file, 135.714.661 B (~0,3% del censimento). Tracciati: 81 build e 5 LaTeX. Ignorati: 1.992 cache, build locali, `opensim.log` root | regole `.gitignore` precise; le dylib in `tools/*/build` solo dopo il confronto tra `plugins/` e i pin |
| Archivio storico | V12, poi 35D, probe e tuning, SNN, plugin non-PI | copia sigillata, restore provato, consumer migrati |
| Refactor | produzione; i 1.732 `.py` pesano 40 MB, quindi il guadagno è solo di leggibilità | per ultimo, con nomi di modulo e LF invariati |

**V12, primo candidato**
- **Dimensioni:** 45.535 file tracciati più 442 ignorati, 1.248.987.108 B. È il 70% dei path di HEAD, ma circa il 9% dei byte tracciati.
- **Beneficio:** checkout, IDE, grep e path lunghi su Windows. Il guadagno in byte è modesto, e `.git` conserva comunque gli oggetti.
- **Condizioni:**
  - chiudere o estrarre il TODO `fit/p2`;
  - lasciare in loco i moduli piatti usati dagli script storici e le regole LF `.gitattributes:28-63`, che servono al restore;
  - delimitare i V12 piatti e gli input H0 byte-pinnati;
  - verificare che non ci siano intersezioni con `protected_sha256`.
- **Forma:** tag o branch d'archivio più copia esterna verificata, poi rimozione da HEAD sul branch di pulizia. Non una rimozione cieca.

**Disco**
- Il volume sta negli ignorati: run dei tre rami 15,56 GB, training 7,46 GB, raw 5,8 GB, duplicati AB06-raw 1,325 GB.
- Si seleziona per lineage dopo un restore provato, senza eliminare in blocco.
- I duplicati si toccano solo dopo aver migrato i riferimenti (per esempio `abx_reference_rrmf81lr`).
- I 2,695 GB di blob ridondanti pesano solo nel working tree, perché Git deduplica.
- I circa 102 KB di duplicati Python sono voluti.

## 3. Sospensione del 39D ≠ dismissione

Il mio «conservare in loco» per V26C e il «non rimuovere ora» per il 39D erano preferenze, non impossibilità tecniche. I vincoli reali sono tre:
- i sigilli verificano path e SHA;
- i pickle dipendono dai nomi di modulo (`asymmetric_rl_module` piatto, `unified_MLP_39.asymmetric_rl_module`);
- il restore deve essere byte-identico, fine riga LF compresi.

Un restore alle posizioni originali li soddisfa tutti e tre. Un branch di pulizia può quindi togliere prove storiche dalla cartella operativa e lasciare la copia sigillata nel checkout o nell'archivio originale. Non c'è obbligo di tenerle lì.

Nella cartella operativa devono invece restare:
- il runtime V26;
- i moduli e le classi importati dal checkpoint (`AsymmetricActorCriticTorchRLModule`, `start_condition_metrics`, `tb_logging`);
- gli asset runtime;
- best47.

**39D.** Si può archiviare, a condizione che package, `runs/` e `validation/` viaggino insieme: la validation cita i run 5.960 volte. In leggibilità costa poco (circa 1,25k file di validation contro i 45,5k di V12), e la sospensione è dichiarata temporanea. Su disco pesa invece 11,8 GB, che si riducono selezionando i run per lineage.

**V26C.** Resta perché è in uso: è la lineage che la nuova fase vuole generalizzare.

**Correzione:** il 35D non è una foglia perfetta. `unified_MLP_39/tests/test_target_availability.py:21` carica `unified_MLP/validation/feasibility39_bundle`. Per ritirare il 35D quel bundle va migrato o conservato, e il test va rieseguito.

## 4. Proposta Codex

È conservativa e utile. Sul piano quantitativo è più solida della mia; la mia è più granulare sui pin.

Concordiamo su:
- perimetro KEEP;
- helper legacy da non toccare;
- restore alle posizioni originali;
- `.git` che non si riduce rimuovendo file da HEAD;
- ordine delle fasi.

Codex aggiunge il parent June23, il divieto di applicare pattern `build` alle campagne e il reload della copia ripristinata.

**Eccezioni da rendere esplicite**
- `.DS_Store` di `models/SEASEA/data`: la mia eccezione è confermata dall'hash live, e Codex l'ha tolta dai candidati.
- `opensim.log` pinnati in `models/AB06_SEASEA_Threadmill/` e in `validation/ab0*`: un pattern generico li colpirebbe.
- Fallback silenzioso al plugin non-PI (`build_abx_seasea_pipeline.py:323-339`, `run_opensim_sea_pipeline.py:66-82`): da valutare prima di toccare `plugins/`.
- Mesh (circa 1,6 MiB): i riferimenti vanno cercati case-insensitive, perché `AM_foot_l` è citato sia come `.stl` sia come `.STL`.

**Test da integrare**
- `materialize_pipeline_bundle.py --verify` e gate J20/J21 dopo ogni archiviazione vicina a V26C;
- `test_target_availability.py` dopo modifiche al 35D;
- per il refactor, rollout deterministico di best47 confrontato con il `rollout_summary.json` pinnato;
- su Windows, path fino a 233 caratteri: senza long path la radice del checkout deve stare entro 14 caratteri.

Reset/step e un piccolo rollout restano smoke di non-regressione, non una qualifica scientifica.

## 5. Limiti

- Codex ha fatto un inventario completo; le verifiche di dipendenza, sue e mie, sono mirate. Nessuno dei due ha fatto un audit semantico dei 153.445 file.
- Io ho letto i pin, non gli hash live. Hash e dimensioni li ha calcolati Codex separatamente.
- Non è stato eseguito nessun bundle, archivio, restore o push.
- Restano aperti: eventuali copie esterne, DLL Windows, contenuto di `backup-pre-filter`, identità delle copie AB07 tracciate, perimetro dei V12 piatti.
