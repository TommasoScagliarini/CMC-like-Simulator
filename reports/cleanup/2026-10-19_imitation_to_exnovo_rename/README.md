# Rinomina delle procedure imitation → ex novo — 2026-10-19

Su richiesta dell'utente, `Trajectory Generator/baseline_MLP/validation/`
è stata rinominata in
[`Trajectory Generator/baseline_MLP/imitation_to_exnovo/`](<../../../Trajectory Generator/baseline_MLP/imitation_to_exnovo/>).
I 1.357 file trasferiti, per 2.814.094.745 byte, sono conservati byte per byte.
Questo comprende 213 sorgenti Python e 51 file `.pkl`.

Restano le cinque cartelle F0, F1, F2R, V26B e V26C. Non sono stati ricreati
file nella radice, fixture eliminate dall'utente o collegamenti simbolici.
Il preset ex novo corrente, best47 e il checkpoint H0 restano invariati.

## Compatibilità dei percorsi

Il resolver centrale riconosce l'esatto vecchio namespace della workspace,
anche nei percorsi salvati macOS, Windows e UNC. Trainer, rollout, loader
delle configurazioni e warm start applicano la corrispondenza agli input,
senza modificare i file letti. I percorsi di output restano invariati.

Le risorse esterne esistenti, le copie annidate nei vecchi snapshot e la
cartella `validation/` del simulatore mantengono la propria identità.
Gli input già eliminati restano mancanti: non sono sostituiti da altri asset.

I protocolli, i sorgenti, i checkpoint, i log e i manifest della workspace
non sono stati riscritti. I loro vecchi percorsi restano dati di provenienza.
L'alias dei componenti correnti non cambia i controlli storici che usano
direttamente `Path` o un `cwd` fissato. Il replay esatto dei gate J20/J21
richiede il layout e i prerequisiti originali, incluso il test di telemetria
già eliminato nella pulizia precedente. Non sono stati cambiati hash o
aggirati controlli per presentare una nuova attestazione scientifica.

Il percorso relativo più lungo dei file trasferiti misura 242 caratteri:
per i workflow Windows storici rimane appropriato un checkout corto o il
supporto ai percorsi lunghi già richiesto dalla pipeline.

## Test e documentazione

Il caso di test dedicato ai due YAML e al detector rimossi manualmente
dall'utente è stato ritirato; le sei prove numeriche del corridoio causale
sono conservate integralmente. Le regressioni della migrazione controllano
checkpoint e YAML J20 reali, hash e portabilità dei percorsi.

I README correnti e `AGENTS.md` descrivono il nuovo nome. Trentanove report
hanno una nota datata; quattro link J20/J21 puntano alla nuova posizione.
Corpi, comandi e hash storici restano preservati. I vecchi registri cleanup
non sono stati riscritti.

## Registro

La verifica SHA-256 dell'intera repository conferma i 1.357 trasferimenti
byte-identici e 29.071 file originali invariati fuori dalla workspace. Le
modifiche riguardano soltanto il resolver, quattro consumer dei percorsi, il
test delle fixture ritirate e 42 documenti; è stato aggiunto un modulo di
regressione. Sono passati 108 test selezionati, 97 controlli del loader e
i tre comandi CLI `--help`. Nessun training o simulazione è stato avviato.
Il commit HEAD è rimasto invariato.

- [File trasferiti e SHA-256](moved_files.json).
- [Documenti aggiornati](document_changes.json).
- [Note e collegamenti dei report](report_changes.json).
- [Verifica finale](verification.json).
- [Test selezionati](targeted_tests.log).
