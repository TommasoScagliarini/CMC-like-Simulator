# Rimozione dei training sperimentali e dei port checkpoint-zero — 2026-10-19

> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../2026-10-19_baseline_validation_removal/README.md).

Su richiesta dell’utente sono state eliminate le quattro cartelle seguenti:

| Cartella | File | Byte |
| --- | ---: | ---: |
| `Trajectory Generator/runs/training/validation/` | 3043 | 335212515 |
| `Trajectory Generator/baseline_MLP/validation/v12r7zero/` | 5 | 122160 |
| `Trajectory Generator/baseline_MLP/validation/v12r8zero/` | 6 | 154263 |
| `Trajectory Generator/baseline_MLP/validation/v12r9zero/` | 6 | 159459 |

Totale: **3060 file, 335648397 byte**.
Sono compresi i checkpoint e i dataset degli esperimenti di luglio,
i sorgenti dei tre port `v12r7zero`, `v12r8zero`, `v12r9zero` e i loro
tre moduli di test dedicati. Questi sorgenti non sono stati ricreati altrove.

## Documentazione conservata

Il solo [archivio di configurazioni e riepiloghi](historical_training_summaries_and_configs.zip)
contiene 31 file originali e occupa 168489 byte.
Non contiene pesi, dataset NPZ, tracce, log estesi o sorgenti Python.
Include la configurazione del pilot del 15 luglio usata come modello nei
contratti storici: è conservata come documento, non come input operativo
a un percorso alternativo inserito nei vecchi script.

La [mappa dei file archiviati](archived_documents.json) conserva percorsi,
dimensioni e hash originali; il [manifest della rimozione](removed_files.json)
registra tutti i file eliminati. La [verifica finale](verification.json)
riporta i controlli di integrità e della pipeline corrente.

## Effetto sulla pipeline corrente

Il simulatore CMC-like, il normale training/rollout baseline e i test
correnti in `validation/test/` non importano questi tre port né leggono
gli esperimenti rimossi. H0 canonico resta nella `validation/critic_warmup/`
del root; il checkpoint imitation del warm-start raw e la best47 restano
nelle rispettive cartelle di training esterne alla rimozione.

## Procedure storiche dipendenti

Sono conservati come sorgenti storici i seguenti namespace, che ora
non sono eseguibili o importabili senza ripristinare i port eliminati:

- `v12r8morph/`: `h0_v12r8_morphology_contract.py`, `run_h0_v12r8_morphology.py`, `test_h0_v12r8_morphology_scaffold.py`.
- `v12r9morph/`: `h0_v12r9_morphology_contract.py`, `run_h0_v12r9_morphology.py`, `test_h0_v12r9_morphology_scaffold.py`.
- `v12r9training/`: `h0_v12r9_training_contract.py`, `validate_h0_v12r9_training.py`, `test_h0_v12r9_training.py`.

In particolare, i tre test storici elencati sopra non sono più
raccoglibili come suite autonoma. Non sono stati aggiunti skip, stub,
sostituti dei port o modifiche ai contratti scientifici per mascherare
l’assenza delle dipendenze. Questi namespace appartengono alla vecchia
filiera V12, superata dal retraining imitativo nativo V26.

Anche gli inventari F0, l’audit V26C J0 e il materializzatore del bundle
citano alcuni checkpoint storici ora rimossi. I loro percorsi e hash
restano nello stato originale; per ripetere quei confronti occorre
ripristinare i relativi input. La ricostruzione con una nuova rete
richiede invece nuove configurazioni, dati e verifiche.

## Procedimento futuro

Restano conservati la [guida V26→V26C](../../user/2026-08-28_guida_ricostruzione_pipeline_exnovo_training_ready_da_imitativo_v26_agosto.md)
e il codice V26C in `Trajectory Generator/baseline_MLP/validation/`.
La guida documenta il metodo storico; i runner fissati sui vecchi
checkpoint non costituiscono una pipeline generica pronta al rilancio.
