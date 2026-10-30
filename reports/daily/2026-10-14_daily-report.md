# Daily Report — 2026-10-14

> Daily ricostruito il 2026-10-29 dai report utente del giorno: `end_day` non
> era stato eseguito. Percorsi e numeri sono quelli del 14 ottobre. Le pulizie
> del 16–19 ottobre hanno poi rimosso, fra l'altro, la filiera V12,
> `unified_MLP/`, `unified_MLP_39/` e `Trajectory Generator/validation/`:
> vedere i registri in [reports/cleanup/](../cleanup/).

**Stato:** audit della codebase concluso e prima pulizia tecnica eseguita e
verificata. Nessun commit o push. Instruction check token: CMC_AGENT_OK_2026.

## Audit della codebase e proposta di pulizia

Report: [audit della codebase e dello snapshot](../user/2026-10-14_audit_pulizia_codebase_e_snapshot.md).

**Snapshot.** Il commit locale `dba42388c` (65.125 percorsi) è avanti di uno
rispetto al remoto GitHub (`578254057`) e non è pubblicato. I file ignorati
(checkpoint, gran parte dei dati, plugin compilati, risultati, plot) non sono
nel commit: la run best47 ha zero file tracciati. Le 31 cancellazioni SNN già
presenti restano recuperabili dal commit. Tre file oltre 100 MiB senza Git LFS
sotto `unified_MLP_39/` bloccherebbero un push ordinario su GitHub.

**Inventario.** 153.445 file per 44,77 GB: 13,65 GB tracciati e 31,12 GB
ignorati. `Trajectory Generator/` pesa 30,66 GB (`unified_MLP_39/` 11,80,
`runs/` 9,88, `baseline_MLP/` 4,81, `unified_MLP/` 2,53, `validation/` 1,64).
Seguono `validation/` root 6,63 GB, `models/` 5,83, `results/` 0,89, `plot/`
0,65 e `reports/` 0,0076.

**Da mantenere.** I 16 Python root del simulatore, più `plot/plotter.py` e
`validation/validate_sim_results.py`, chiamati da `main.py` via subprocess. I
sette moduli condivisi del generatore e i 30 Python della baseline. Il
constructor pickle di best47 richiede che
`asymmetric_rl_module.AsymmetricActorCriticTorchRLModule` resti importabile.
`rl_module_best/` basta per l'inferenza; per riprendere PPO serve il
checkpoint completo Algorithm/Learner. Dipendenze nascoste: i default
`CMC_Actuators.xml` e `CMC_Tasks` in `models/SEASEA/data/`, il profilo detector
V25 di produzione dentro `validation/` e il checkpoint di warm start in
`validation/critic_warmup/`.

**Candidati alla pulizia.**
- Prodotti tecnici: circa 136 MB fra cache, `opensim.log` root (78 MB), build
  e intermedi LaTeX.
- Codice opzionale da ritirare con migrazione: il vecchio `commands.txt` SNN,
  i rami `unified_MLP*`, i moduli primary split, i tool di tuning.
- Duplicati: `AB06-raw` e `AB06_SEASEA-raw` hanno 1.719 file identici
  (1,325 GB).
- Filiera V12: 45.535 file tracciati, circa 1,25 GB e il 70% dei percorsi di
  HEAD; è il primo candidato all'archivio.
- Rollout (15,56 GB) e training (7,46 GB): da selezionare, non da rimuovere
  in blocco.

L'audit propone anche un indice dei TODO con ID stabili, realizzato il
2026-10-29 con [TODO_BACKLOG.md](../TODO_BACKLOG.md).

**Strategia proposta in sei passi.**
1. Proteggere il commit e i file ignorati, con un restore provato.
2. Rimuovere i prodotti tecnici.
3. Consolidare comandi e indice degli entrypoint.
4. Separare il checkout operativo dagli archivi.
5. Canonizzare i duplicati.
6. Eseguire verifiche di non regressione, incluso Windows.

**Revisione.** Claude Opus 5.5 con effort max ha svolto l'audit source
indipendente e il confronto finale con Codex, con modello verificato dalla
ricevuta. Sono state riconciliate due eccezioni: il `.DS_Store` di `SEASEA` è
pinnato nelle prove AB07, e il `feasibility39_bundle` della 35D serve al test
39D. Evidenze in [reports/cleanup/2026-10-14/](../cleanup/2026-10-14/).

## Prima pulizia della codebase

Report: [prima pulizia e verifica di integrità](../user/2026-10-14_prima_pulizia_codebase_e_integrita.md).

Eseguita dopo la dichiarazione dell'utente di avere copiato la cartella su un
disco esterno; il backup non è stato ispezionato. Rimossi **2.158 file
generati (135,73 MB)** e 127 directory vuote:

| Gruppo | File | Byte |
|---|---:|---:|
| Cache Python e metadati | 1.988 | 48.267.393 |
| Prodotti delle quattro build tecniche | 164 | 9.204.323 |
| `opensim.log` root | 1 | 78.159.231 |
| Intermedi LaTeX di `paper/main` | 5 | 98.046 |

L'unico file esistente modificato è `.gitignore`, con sette regole precise
(`/build/`, `/build_online_grf/`, intermedi LaTeX). Sono stati conservati tre
`.DS_Store` vincolati da hash o provenance. Sorgenti C++, `CMakeLists.txt` e
librerie in `plugins/` non sono stati toccati.

**Verifiche.**
- SHA256 di 151.319 file conservati: cambia solo `.gitignore`.
- 86 ancore di sorgenti, modelli, profili, plugin e checkpoint: PASS.
- Reload di best47 (actor 35, full 84, action 2): PASS.
- Smoke OpenSim nativo macOS di due segmenti da 0,01 s: PASS, con residuo ID
  massimo 2,64e-13 N·m. Non è un episodio completo e usa i default FSM v2 del
  valutatore.
- Bundle V26C: archivio PASS; suite 17/19, con due problemi preesistenti
  (drift di `v26c_j1_collect.py` e tre file aggiunti dopo il registro di
  agosto).

Git: HEAD resta `dba42388c`; le 86 cancellazioni tracciate (81 prodotti CMake
e 5 intermedi LaTeX) vanno separate dalle 31 SNN preesistenti in un futuro
commit selettivo. Evidenze in
[reports/cleanup/2026-10-14_phase1/](../cleanup/2026-10-14_phase1/).

## TODO

Nessun TODO esplicito nei report del giorno, quindi nessuna voce aggiunta al
[registro](../TODO_BACKLOG.md). I report indicano come attività separate:
- protezione del commit e dei file ignorati, con restore verificato, prima di
  nuove rimozioni;
- gestione dei tre file oltre 100 MiB prima di un push;
- riconciliazione del registro V26C congelato;
- commit selettivo che separi le cancellazioni SNN.
