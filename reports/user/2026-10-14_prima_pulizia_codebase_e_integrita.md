# 2026-10-14 — Prima pulizia della codebase e verifica di integrità

> Aggiornamento della struttura del 2026-10-19: le procedure conservate nella
> precedente `Trajectory Generator/baseline_MLP/validation/` sono ora in
> [`imitation_to_exnovo/`](<../../Trajectory Generator/baseline_MLP/imitation_to_exnovo/>).
> I percorsi e gli hash nel corpo restano quelli del documento originale.
> Mappa della rinomina, compatibilità e dipendenze già mancanti sono nel
> [registro della rinomina](../cleanup/2026-10-19_imitation_to_exnovo_rename/README.md).


> Aggiornamento del 2026-10-19: `Trajectory Generator/baseline_MLP/experimental_configs/`
> è stata rimossa. Due preset sono conservati come fixture dei test; gli altri
> preset e i comandi che li selezionavano sono storici e richiedono ripristino.
> La configurazione V26C corrente resta `training_exnovo_cfg.yaml`. Vedere il
> [registro della rimozione](../cleanup/2026-10-19_experimental_configs_removal/README.md).

La prima pulizia tecnica è stata eseguita dopo la dichiarazione dell'utente di avere copiato tutta la cartella su un disco esterno. Rimossi **2.158 file generati, 135.728.993 byte (135,73 MB decimali)** e **127 directory rimaste vuote**. Nessun cambiamento ai sorgenti del simulatore, al plugin SEA, alla policy, ai dati, ai checkpoint o alle prove scientifiche conservate. L'unico file esistente modificato è `.gitignore`; questo report e le evidenze sono nuovi documenti dell'intervento.

Il confronto SHA256 prima/dopo ha verificato **151.319 file conservati, 44.636.575.150 byte**: 151.318 identici e la sola variazione prevista di `.gitignore`. Nessun altro file perso, aggiunto, modificato o rigenerato. Reload best47 e smoke OpenSim nativo di due passi superati. La suite del bundle storico V26C conserva due problemi già presenti prima della pulizia: il risultato complessivo va quindi distinto dall'integrità dei file mantenuti.

## Problema e strategia

La repository contiene mesi di cache Python, metadati Finder, log e prodotti di build, insieme ad archivi scientifici con vincoli di percorso e hash. La prima fase elimina solo gli artefatti tecnici classificati nell'[audit precedente](2026-10-14_audit_pulizia_codebase_e_snapshot.md), con verifica aggiornata dei riferimenti prima della rimozione.

Ogni target è stato registrato con percorso, dimensione e SHA256, poi ricontrollato immediatamente prima della cancellazione. La rimozione ha usato singoli file e `rmdir` sulle sole directory tecniche vuote, senza cancellazioni ricorsive indiscriminate. Un journal registra ogni file rimosso. Il backup esterno è dichiarato dall'utente e non è stato ispezionato da questa sessione.

## File rimossi e modifica preventiva

| Gruppo, senza sovrapposizioni | File | Byte |
|---|---:|---:|
| Cache Python e metadati incidentali fuori dalle quattro build | 1.988 | 48.267.393 |
| Prodotti e cache nelle quattro build tecniche | 164 | 9.204.323 |
| `opensim.log` del root | 1 | 78.159.231 |
| Intermedi LaTeX di `paper/main` | 5 | 98.046 |
| **Totale** | **2.158** | **135.728.993** |

Le quattro build sono `/build/`, `/build_online_grf/`, `/tools/sea_plugin_relative_d/build/` e `/tools/online_grf_contact/build/`. I sorgenti C++, i `CMakeLists.txt` e le librerie runtime in `plugins/` sono conservati. Le due librerie macOS presenti nelle build avevano copie byte-identiche in `plugins/`, che è il percorso caricato dalla configurazione.

Rimossi solo `paper/main.aux`, `.fdb_latexmk`, `.fls`, `.log`, `.synctex.gz`: restano `main.tex`, `main.pdf`, le sezioni e i documenti in `paper/aux/`. I log nei run, nei modelli e nelle verifiche scientifiche sono conservati.

In [.gitignore](../../.gitignore) sono state aggiunte sette regole precise: `/build/`, `/build_online_grf/` e i cinque intermedi LaTeX. Le build dei tools erano già ignorate. Verificati otto casi: queste regole escludono i prodotti tecnici, mentre restano visibili i sorgenti, il manoscritto e le directory scientifiche annidate chiamate `build`.

## Eccezioni conservate

La revisione finale dei riferimenti ha individuato tre `.DS_Store` da conservare:

- `models/SEASEA/data/.DS_Store`, già escluso nell'audit perché vincolato da hash;
- `models/AB06_SEASEA_Threadmill/.DS_Store`, vincolato dalla provenance della verifica indipendente AB06/AB07;
- `models/AB06-raw/10_09_18/static/.DS_Store`, nominato nell'inventario della verifica del montaggio AB07, conservato per non alterare tale inventario.

I loro hash sono invariati. Il perimetro finale differisce dalla prima matrice dell'audit perché esclude due ulteriori metadati protetti, include un nuovo metadato nella build e usa le dimensioni correnti dei file. Non sono stati cancellati file per il solo fatto di avere un nome generico come `build`, `.DS_Store` o `opensim.log`. Sette `.DS_Store` sotto i rami `unified_MLP*`, comparsi dopo il censimento e fuori dall'elenco operativo dell'intervento, sono conservati.

Alcuni bytecode senza sorgente corrispondente, ad esempio `unified_MLP/__pycache__/grf_defaults.cpython-310.pyc`, erano tracce storiche residuali: non erano importabili da Python 3 da quel percorso senza il `.py`. La loro copia e i vecchi dettagli della compilazione (`CMakeCache.txt`, `flags.make`, `link.txt`) rimangono nel backup esterno dichiarato; i binari runtime conservati sono identici. La leggibilità della cache non è stata usata come sostituto dei sorgenti o come prova di ricompilazione.

## Verifiche eseguite

| Verifica | Risultato e significato |
|---|---|
| SHA256 completo dei file mantenuti | PASS: 151.319 file verificati, unica modifica prevista `.gitignore` |
| 86 ancore di sorgenti, modelli, profili, plugin e checkpoint | PASS, ricontrollate anche da un secondo agente |
| Manifest delle rimozioni e journal | PASS: stessi 2.158 path unici, totale byte concordante, nessun target residuo |
| Regole `.gitignore` | PASS: otto casi, incluse esclusioni precise e conservazione delle build scientifiche annidate |
| Reload nativo `rl_module_best` best47 | PASS: classe `AsymmetricActorCriticTorchRLModule`, actor 35/full 84/action 2, inferenza finita, risultati uguali tra input completo e prefisso actor |
| Smoke OpenSim/macOS arm64 | PASS: caricamento modello, plugin SEA e GRF, reset e due segmenti fisici da 0,01 s; residuo ID massimo 2,64e-13 N·m |
| Archivio objects e control del bundle V26C | PASS: 692 oggetti, 257.138.708 byte, 1.888 entry sorgente |
| Manifest V26C in_place | 1.356/1.357 corrispondono al registro di agosto; unico drift già presente nella baseline e nel commit HEAD |
| Suite strutturale V26C | 17/19 PASS; un failure e un error dovuti alle discrepanze storiche descritte sotto |
| Delta Git e `git diff --check` | PASS: sola modifica `.gitignore`, 86 nuove cancellazioni generate, 31 cancellazioni SNN preesistenti separate |

Reload eseguito con `/opt/anaconda3/envs/envCMC-rllib/bin/python` (Python 3.10.20, Torch 2.10.0, Ray 2.55.1), senza cluster Ray. Lo smoke usa il checkpoint `MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/rl_module_best`, il setup AB06 PI, azioni deterministiche, seed 123, offset 1,956870983805102 s, durata nominale episodio 5 s e limite di due step. Gli output sono stati scritti nella cartella temporanea delle verifiche, senza STO né plot.

Lo smoke dimostra caricamento, reset e 20 ms di integrazione: non è un episodio completo di 5 s né una replica scientifica della campagna best47. Il valutatore ha applicato i propri default `legacy_events_v1`/FSM v2; non è una qualifica FSM v3. `end_reason` è nullo e `terminated`/`truncated` sono false. Reward, reserve e comportamento di cammino non sono stati qualificati. Il confronto di inferenza fra prefisso e input completo usa valori tutti nulli: verifica caricamento e inferenza finita, senza dimostrare indipendenza da osservazioni privilegiate variabili. Non sono state eseguite prove native su Windows né una ricompilazione dei plugin.

Rispetto al piano preliminare, la durata nominale dello smoke è stata portata da 0,02 a 5 s, mantenendo `max_steps=2` e impostando l'offset operativo: la durata simulata effettiva resta circa 0,02 s. Questo evita di alterare il contratto nominale dell'episodio. L'output della verifica dedicata delle 86 ancore è stato salvato anche dopo la revisione finale; 84 ancore hanno pin storici, due sono ancore operative senza un pin storico. Il confronto completo SHA include tutte e 86.

## Discrepanze V26C preesistenti

Il comando `materialize_pipeline_bundle.py --verify --full` falliva già prima della rimozione su:

`Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j1_collect.py`

Il registro di agosto richiede 55.428 byte e SHA256 `83862a8684d1970fc7f205be43a71040ec0a655f545e7c4f4607ac8f38c6691d`; il file corrente è di 55.320 byte, SHA256 `f264342b978db1879620c6fdf803a6662f252ee82f61accc7ff3b012aaff6bb8`. Questi byte coincidono sia con la baseline precedente alla pulizia sia con il blob nel commit corrente. Il file non è stato modificato dall'intervento.

Il test `test_audited_source_closure_is_exact` trova inoltre tre file aggiunti dopo il registro di agosto, tutti già presenti e byte-identici nella baseline di questa pulizia:

- `Trajectory Generator/baseline_MLP/experimental_configs/morphology_joint_split_equal_5p0_2026-09-01.json`;
- `Trajectory Generator/baseline_MLP/experimental_configs/morphology_joint_split_equal_1p05_2026-08-31.json`;
- `reports/user/2026-08-28_rollout_best_exnovo_v26c_j20_plot_canonici_e_corridor_morfologico.md`.

Tutti i 1.357 file in_place sono rimasti identici alla baseline. Gli oggetti archiviati e i control sono validi. I vecchi manifest e i sorgenti storici sono conservati senza riscriverli: riconciliare il registro congelato con le modifiche successive richiede un intervento separato sulla provenance, non una correzione della pulizia.

## Stato Git e collaborazione

HEAD resta `dba42388c32a441a5543f20881612a52781d7a99`. Le 86 rimozioni tracciate dell'intervento sono 81 prodotti CMake e cinque intermedi LaTeX. Le altre 31 cancellazioni mostrate da Git appartengono a `Trajectory Generator/Prosthesis_SNN/` ed erano già presenti all'inizio della sessione. Nessun commit, push o modifica all'indice è stato eseguito.

Due agenti hanno revisionato separatamente rimozioni e verifiche. **Claude Opus 5.5 con effort `max`** ha completato la revisione in sola lettura, senza fallback: la ricevuta conferma il modello effettivo `claude-opus-5-5`. Ha confermato il perimetro e l'assenza di perdita dei file conservati o di dipendenze runtime rimosse, chiedendo di rendere persistenti le evidenze e precisare i limiti dei test. Le osservazioni sono state integrate qui e nella sintesi delle verifiche.

Dopo la revisione sono stati eliminati, con soli `rmdir`, 24 ulteriori scheletri di directory vuote rimasti nelle build: totale 127 directory rimosse, senza ulteriori file cancellati. Le quattro build tecniche ora sono assenti; il journal delle 2.158 cancellazioni di file è invariato. Questa rifinitura è registrata in `post_review_empty_directory_cleanup.json`.

Le nuove regole di ignore impediscono di aggiungere prodotti futuri non tracciati. Gli 86 path già presenti nell'indice rimangono tracciati finché le cancellazioni non vengono committate. Un futuro commit va preparato selettivamente per tenere separate le 31 cancellazioni SNN preesistenti.

## Evidenze riproducibili

L'elenco puntuale delle cancellazioni, gli hash prima/dopo, i risultati e la revisione Claude sono conservati in [reports/cleanup/2026-10-14_phase1/](../cleanup/2026-10-14_phase1/). Il manifest completo precedente alla rimozione e il journal sono compressi per contenere il peso della documentazione. [artifact_manifest.json](../cleanup/2026-10-14_phase1/artifact_manifest.json) registra dimensioni e SHA256 delle evidenze e di questo report; [delivery_checks.json](../cleanup/2026-10-14_phase1/delivery_checks.json) registra i controlli della consegna.

Questa fase è conclusa: nessuna perdita o modifica imprevista individuata nei file mantenuti, checkpoint e integrazione breve funzionanti, registri V26C storici con discrepanze preesistenti documentate. La rimozione di codice storico o l'archiviazione di campagne scientifiche resta fuori da questo primo intervento.
