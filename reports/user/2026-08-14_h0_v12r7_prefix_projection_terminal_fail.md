# H0 V12R7 — terminal fail per proiezione errata del prefisso V26

Data: 2026-08-14  
Stato: **V12R7 TERMINAL FAIL; evidenza fisica conservata; successor V12R8 necessario**

## Problema

Il protocollo congelato V12R7 ha eseguito una sola volta il primo caso critico
`deterministic_offset_plus_0p20`. Il rollout ha riprodotto esattamente il
terminale fisico R6 dopo 179 step, ma la pipeline si e chiusa con
`FAIL_H0_V12R7_RECOVERY_PIPELINE_TERMINAL` prima dell'etichettatura offline.

La causa non e una nuova anomalia del detector e non e un mismatch della
riproduzione fisica. Il gate R7 ha letto tre contatori V26 e il
`target_contract_id` dalla proiezione top-level del summary, dove questi campi
non vengono pubblicati. I contatori richiesti sono invece presenti dentro
`binary_phase_event_gate`. Il controllo top-level li ha quindi interpretati
come mancanti e ha prodotto un falso negativo:

- `contract_integrity=false`;
- `detector_active=false`;
- `zero_detector_anomalies=false`.

Il valore `binary_phase_event_gate.passed=false` e coerente con un episodio
fisicamente troncato prima dell'orizzonte completo e non invalida, da solo, il
prefisso. La semantica gia coperta dai test V12R3 distingue infatti la qualifica
di un episodio completo dall'integrita causale del prefisso.

## Evidenza che il prefisso e integro

Il rollout R7 ha prodotto:

- 179 step e 1790 campioni raw, cioe esattamente 10 campioni per control step;
- `binary_event_prefix_integrity.passed=true`;
- zero `duplicate_event_count`, `out_of_order_event_count`,
  `left_non_v26_source_count`, `fallback_count` e `hard_invalid_count` nella
  struttura V26 annidata;
- due eventi V26 coerenti, `toe_off` e `heel_strike`;
- `r6_plus_reproduction_audit.passed=true`, con 179/179 righe e tutti i campi
  runtime condivisi byte-equivalenti allo storico R6;
- trace pura, senza teacher, blending, safety latch o update online;
- osservazioni actor e penetrazione byte-esatte tra trace e replay;
- terminale fisico `grf_penetration` a `0.025790675039579398 m`, uguale al caso
  critico di riferimento.

Il test preesistente
`v12r3/test_h0_primary_split_v12r3_execution.py::test_clean_v26_physical_prefix_is_integrity_valid_and_recoverable`
formalizza proprio questo caso: event gate completo non passato, prefix
integrity passata e prefisso recuperabile per la raccolta dati.

## Strategia e soluzione

V12R7 resta immutabile e terminale: il protocollo vieta retry, resume e patch
post-freeze. La soluzione viene implementata in un nuovo namespace V12R8:

1. attestare hash e stato terminale degli artefatti R7;
2. applicare un adjudicator puro al prefisso `+0.20`, leggendo l'evidenza V26
   annidata e `binary_event_prefix_integrity`;
3. se e solo se l'adjudication passa, etichettare offline quel replay nel
   namespace R8, senza ripetere il rollout fisico;
4. raccogliere ed etichettare gli altri cinque casi R8 con una proiezione V26
   esplicita e corretta;
5. eseguire l'unico fit W512 previsto, congelare il solo candidato risultante e
   svolgere i sei rollout development;
6. aprire Q3 soltanto dopo un terminal PASS R8.

Questa soluzione non rilassa i requisiti fisici o detector: corregge soltanto
il punto da cui il gate legge evidenza gia presente e verificabile.

## File e artefatti coinvolti

Artefatti R7 congelati e non modificati:

- `Trajectory Generator/baseline_MLP/validation/v12r7/h0_v12r7_recovery_protocol_freeze.json`
  — SHA-256 `ea6d4b795c33696f6fbdf54557e907b8a4ff819dbfdb1cc0edf59f69d07ea32b`;
- `Trajectory Generator/baseline_MLP/validation/v12r7/h0_v12r7_recovery_execution_lock.json`
  — SHA-256 `fe3fc9a0f4f06d3df83cd0bfe99eadf99d77ea21074053815fad47ef6e2fab1c`;
- `Trajectory Generator/baseline_MLP/validation/v12r7/h0_v12r7_run_20260814/pipeline_ledger.json`
  — SHA-256 `8c7761d09625241a80311535466c4acc2bb70f62fd895d1cb1b9627fc6292ae8`;
- summary `+0.20` — SHA-256
  `b07d25996e144f1102dd98e483d5912903e06f400dc52ccb4db5ce5e4bfd399b`;
- trace `+0.20` — SHA-256
  `d259b9a69ab63c3dab354ad7e228d628c58ee9fd5c972e64d8f95dded3483688`;
- replay `+0.20` — SHA-256
  `10b2b9e55364aeb3c4070c77dd8cc1cff12e3bd144da0ec916d7a739b69a14db`;
- gate R7 `+0.20` — SHA-256
  `d6147af3404b1d8386718f2956de51fbb3efde5a5754d63ffe430607916a03f0`;
- receipt R7 `+0.20` — SHA-256
  `027a99250cc050a5c771fd9bb6056f17177779c051b7934ca88ee8a7f0d84f09`.

Il successor vive esclusivamente in
`Trajectory Generator/baseline_MLP/validation/v12r8/`.

## Test e verifiche eseguiti

- verifica read-only degli hash SHA-256 degli artefatti R7;
- confronto 179/179 con lo storico R6: PASS;
- audit byte-esatto trace/replay per osservazioni actor e penetrazione: PASS;
- audit `binary_event_prefix_integrity`: PASS, 1790/1790 campioni;
- audit dei cinque contatori V26 annidati: tutti zero;
- audit assenza teacher/blending/latch/update: PASS;
- confronto con la semantica e i test V12R3 per prefissi fisicamente troncati:
  coerente.

## TODO propagato

- completare freeze ed esecuzione one-shot V12R8;
- dopo PASS R8, eseguire Q3 weight=0 con detector V26 attivo;
- creare e validare il checkpoint-zero imitativo;
- eseguire A/B del Morphology Corridor con peso positivo `0.0025`;
- pubblicare il comando finale da `--resume-from` soltanto dopo tutti i PASS.
