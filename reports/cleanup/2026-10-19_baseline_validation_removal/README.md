# Pulizia della validazione della rete — 2026-10-19

Su richiesta dell’utente sono stati rimossi **46.458 file, 1.975.509.302 byte**.
La rimozione comprende:

- tutti i 25 file direttamente nella radice di `Trajectory Generator/baseline_MLP/validation/`, inclusi README, regole V12 e test;
- le 28 directory `v12*` e `h0_v12r1_run_20260809/` (conteggio complessivo del gruppo);
- `pipeline_bundle_v26c_2026-08-28/`;
- `v26c_sigma_exploration_smoke_2026-08-31/` e il suo analizzatore nella radice.

## Materiali conservati

Restano byte-identici la workspace `v26c_july_replica_2026-08-26/`, i bridge
F0/F1/F2R/V26B e le fixture morphology. Il codice operativo, la configurazione
ex novo, i checkpoint best47/H0, i modelli e i plugin non sono coinvolti.
I test della radice rimossi non sono stati copiati in altre cartelle.

La documentazione corrente è nel [README baseline_MLP](<../../../Trajectory Generator/baseline_MLP/README.md>).
Non è stato ricreato un README nella radice di `baseline_MLP/validation/`.

## Limiti della ricostruzione storica

Il bundle e la campagna sigma non sono più disponibili. I loro comandi, risultati
e confronti riportati nei documenti precedenti descrivono lo stato dell’epoca.
Per ripeterli occorrono un ripristino dal backup/Git o nuove elaborazioni.

La workspace V26C e il checkpoint J20 sono conservati, ma i gate storici J20/J21
e gli audit di restore richiedono ancora i byte originali del test rimosso
`test_training_health_telemetry.py`. Per rieseguire esattamente quei gate occorre
ripristinare quel prerequisito. I pin, i receipt, le attestazioni e i sorgenti
della workspace non sono stati modificati né aggirati.

Gli altri test conservati continuano a verificare il runtime corrente. Non è
stato avviato alcun training né eseguita una simulazione.

## Inventari e riferimenti

La verifica completa SHA-256 conferma che sono stati rimossi soltanto i
46.458 file previsti. Restano invariati 30.391 file originali, inclusi tutti
i 355 sorgenti Python conservati. Le sole modifiche ai file conservati sono
79 annotazioni documentali e i due README correnti. Sono passati 102 test
selezionati e i tre controlli CLI `--help` di simulatore, trainer e rollout.
Il commit HEAD è rimasto invariato; non sono stati creati commit.

- [File rimossi con dimensioni e SHA-256](removed_files.json).
- [Perimetro della rimozione](targets.json).
- [Collegamenti aggiornati](report_links.json).
- [Documenti annotati](document_updates.json).
- [Verifica finale](verification.json).
- [Test selezionati](targeted_tests.log).

I documenti storici hanno una nota datata. Sono stati aggiornati soltanto i
collegamenti ai file rimossi; il resto del testo e dei verdetti precedenti è
conservato. I riferimenti nei manifest JSON congelati restano dati storici.

## Destinazioni dei collegamenti ritirati

<a id="artifact-38c36a11f677b90c"></a>
### `Trajectory Generator/baseline_MLP/validation/analyze_v26c_sigma_exploration_smoke.py`

File rimosso da questa pulizia. Il collegamento originale è conservato nel registro.

<a id="artifact-8c169d5bbe44ae69"></a>
### `Trajectory Generator/baseline_MLP/validation/pipeline_bundle_v26c_2026-08-28/README.md`

File rimosso da questa pulizia. Il collegamento originale è conservato nel registro.

<a id="artifact-4a48da44ff7e42d2"></a>
### `Trajectory Generator/baseline_MLP/validation/pipeline_bundle_v26c_2026-08-28/bundle_manifest.json`

File rimosso da questa pulizia. Il collegamento originale è conservato nel registro.

<a id="artifact-7702ec2d8a4d2900"></a>
### `Trajectory Generator/baseline_MLP/validation/v26c_sigma_exploration_smoke_2026-08-31/rollouts/iso_0p010_seed123_nominal/rollout_summary.json`

File rimosso da questa pulizia. Il collegamento originale è conservato nel registro.

