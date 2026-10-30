# Rimozione dei preset sperimentali — 2026-10-19

> Aggiornamento successivo del 2026-10-19: i riferimenti di questo documento
> alla filiera V12, al bundle V26C, alla campagna sigma o ai file nella radice
> di `baseline_MLP/validation/` sono storici: quei materiali sono stati rimossi.
> La workspace V26C e i suoi checkpoint sono conservati. Per rieseguire
> le procedure con input o prerequisiti rimossi occorre il loro ripristino.
> Vedere il [registro della pulizia](../2026-10-19_baseline_validation_removal/README.md).

La cartella `Trajectory Generator/baseline_MLP/experimental_configs/` è stata
eliminata su richiesta dell'utente. Conteneva nove file per 26.450 byte:
due preset sono stati trasferiti byte per byte tra le fixture dei test;
gli altri sette file sono stati rimossi.

La configurazione operativa rimane
[`training_exnovo_cfg.yaml`](<../../../Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml>),
già allineata a V26C/best47. I sorgenti del training e del rollout, i pesi,
il checkpoint H0 e i plugin restano invariati da questa rimozione.

## Fixture conservate

I due YAML candidate sono ora sotto
[`validation/fixtures/morphology/`](<../../../Trajectory Generator/baseline_MLP/validation/fixtures/morphology/>).
Sono contratti per i test, non preset operativi:

- `morphology_event_anchored_causal_candidate.yaml`: variante storica V17.
- `morphology_event_anchored_causal_v26_candidate.yaml`: candidato V26 promosso
  nell'agosto 2026, con il suo peso storico `0.0025`.

Nella stessa directory è conservato il detector V17 originale da 2.465 byte,
recuperato da Git e verificato tramite lo SHA-256 fissato nel candidato.
Non è stato ripristinato tra i profili operativi. I test continuano a
verificare hash, contratti e protezione Q2; distinguono i pesi storici dal
default corrente best47 (`5.0` per entrambi i giunti).

## Riferimenti storici

Gli inventari F0, i contratti V12 Q3/R4-zero e il materializzatore originale
del bundle conservano i percorsi e i pin storici. Per riaprire quelle
procedure occorrono gli originali corrispondenti da Git o dal backup.
Non sono stati cambiati hash o manifest per presentare le fixture spostate
come gli input originali. Gli oggetti già conservati nel bundle V26C restano
verificabili senza il confronto opzionale con le sorgenti live.

Gli undici report interessati hanno una nota datata; il testo originale è
conservato integralmente. I README correnti puntano alle nuove fixture e
descrivono il ritiro dei preset.

## Registro e verifiche

- [File rimossi e trasferiti](removed_files.json).
- [Hash e origine delle fixture](test_fixtures.json).
- [Annotazioni dei report](report_annotations.json).
- [Verifica finale](verification.json).
