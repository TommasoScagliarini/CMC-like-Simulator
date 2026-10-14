# Diagnostica dei pesi V26 nella unified39 — 23 settembre 2026

Esperimento autorizzato dall'utente: riutilizzare la policy best87 del training
imitativo del 20 agosto nella struttura unified39, mantenendo il riferimento
destro storico e la FSM v3 attuale. Nessun apprendimento, modifica SEA o
sovrascrittura dei checkpoint originali.

## Metodo

1. Caricare l'export storico di inferenza (solo actor).
2. Costruire il modulo unified39 in modalità `imitation`, sempre visibile.
3. Permutare le colonne del primo layer: nuovo → storico =
   `[0, 1, 6, ..., 38, 2, 3, 4, 5]`. Applicare la permutazione a entrambi gli
   alias serializzati del layer. I bias e tutti i layer successivi sono copiati.
4. Verificare media e logstd su 500 osservazioni storiche e 4.096 campioni
   sintetici, verificare le azioni archiviate e ricaricare l'export prodotto.
5. Nel solo ambiente diagnostico usare il template periodico destro: shift
   ginocchio 0,465, caviglia 0,452; clock actor destro non sfasato; velocità
   in rad/s senza divisione per 10. La reward usa lo stesso target e le sue
   escursioni. Le altre impostazioni sono quelle della unified39 allineata.
6. Eseguire un rollout deterministico da 12,99 s, seed 123, massimo 500 passi.
   A ogni passo confrontare nuovamente le uscite della policy convertita con
   quelle originali sugli stessi segnali correnti rimappati.
7. Verificare gli artefatti, confrontare con il rollout originale V26 e best72,
   salvare i sette plot canonici nel root `plot/`.

Il suffisso privilegiato resta di 49 ingressi, per 88 totali. Non si trasferisce
il critic, assente nell'export storico e non usato per l'inferenza. Questo test
non prepara un checkpoint di training né trasferisce Adam.

## Contratto separato

`converted_module/diagnostic_contract.json` identifica esplicitamente RIGHT,
nomi e ordine degli ingressi, fonte e hash dei pesi. Non viene creato un
`unified_contract.json` sinistro: il normale validatore di produzione rifiuta
questo export. I validatori di produzione non sono stati modificati.

Le voci `gait_phase` e `gait_raw_phase` del reset grezzo appartengono al clock
ereditato, disabilitato. Il clock destro effettivo viene introdotto dal wrapper;
`artifact_verification.json` ne documenta la fase iniziale verificata contro
sin/cos dell'actor e fase target più shift, preservando il reset originale.

## Riproduzione e output

Usare l'ambiente Python del progetto, dalla root:

```text
python "Trajectory Generator/unified_MLP_39/validation/v26_weight_transfer_2026-09-23/probe.py" prepare
python "Trajectory Generator/unified_MLP_39/validation/v26_weight_transfer_2026-09-23/probe.py" rollout
python "Trajectory Generator/unified_MLP_39/validation/v26_weight_transfer_2026-09-23/verify_artifacts.py"
python "Trajectory Generator/unified_MLP_39/validation/v26_weight_transfer_2026-09-23/analyze.py" --render
```

`prepare` e `rollout` rifiutano di sovrascrivere rispettivamente un export e un
risultato già esistenti. Il rollout è sorvegliato dal watchdog multipiattaforma.

Artefatti: `offline_equivalence.json`, `production_contract_guard.json`,
`input_hashes.json`, `artifact_verification.json`, `metrics.json`,
`analysis_input_hashes.json`, `canonical_plot_receipt.json`. Il rollout completo
è in `unified_MLP_39/runs/rollout/V26_best87_in_unified39_right_fsmv3_2026-09-23`.

Un singolo rollout deterministico può verificare il riuso della soluzione
storica nel contenitore attuale; non misura la robustezza esplorativa e non
dimostra equivalenza statistica di due training. Le differenze rispetto a
best72 includono sia la policy sia il target sinistro, quindi non isolano
l'effetto causale del solo target.
