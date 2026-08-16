# Addendum H0 primary split V5 — nuova lineage full-mean

## Motivo della nuova lineage

V3 resta chiuso con `FAIL_H0_PRIMARY_SPLIT_V3_OFFLINE`: il candidato limitato
alle colonne 10/11 non ha superato i gate train-only. Il corpus V3 dei seed
123/124 resta invece PASS, congelato e riutilizzabile.

V4 resta chiuso prima dell'esecuzione con
`FAIL_H0_PRIMARY_SPLIT_V4_PREEXECUTION_INTEGRITY`. Il runner è cambiato dopo il
freeze del lock e il receipt preflight bloccato è diventato stale; la verifica
del lock ha fallito prima di creare attempt claim o run root. Il receipt
`validation/h0_primary_grf_split_v4_preexecution_failure.json` attesta:

- zero candidati e zero actor/critic/PPO update;
- nessun accesso semantico al seed 125;
- nessun run V4 avviato;
- nessun retry V4 autorizzato.

V5 è quindi un protocollo nuovo e no-clobber. Non riusa candidati V3/V4, non
modifica né riscrive alcun artefatto delle lineage precedenti e congela come
input sia il failure receipt V4 sia il corpus/ledger V3.

## Candidato unico V5

Il design resta quello train-only già verificato:

- sorgente: H0 originale;
- scope: intera rete delle medie (`trainable_first_layer_features=None`);
- logstd congelata bit-exact;
- 400 epoche con checkpoint finale fisso;
- seed 123, batch 128, learning rate `5e-5`, anchor `1e-2`;
- validation fraction e patience pari a zero;
- un solo actor update candidate, zero critic update e zero PPO update.

L'evidenza di design su seed 123/124 è RMSE/max student
`0.0030197/0.02477` e RMSE/max teacher `0.0030618/0.02053`. Non è un holdout,
non seleziona ulteriori candidati e non autorizza retuning.

## Gate offline

Il candidato V5 deve rispettare gli stessi gate:

- student RMSE `<= 0.01`, max error `<= 0.10`, riduzione RMSE `>= 50%`;
- teacher RMSE `<= 0.005`, max error `<= 0.05`;
- logits finiti e medie entro `[-1, 1]`;
- 400 epoche fixed-final e tutte le 2000 righe train utilizzate;
- hidden e output delle medie effettivamente modificati;
- tensori actor finiti, alias encoder bit-exact e clock disabilitati a zero;
- parametri/output logstd bit-exact;
- non-actor esatto o assente nel modulo inference-only;
- save/reload actor esatto;
- sorgente H0 originale e nessun candidato V3/V4 riutilizzato.

Ogni manifest, receipt e freeze propaga un record exact del failure V4 e del
ledger/corpus V3. Un FAIL offline è terminale e non apre seed 125.

## Holdout procedurale 125

Il freezer può soltanto hashare trace e summary del seed 125 come byte opachi.
Dopo il PASS offline, il runner:

1. pubblica `adaptation/candidate_freeze.json`;
2. pubblica `holdout_access_claim.json`;
3. ribinda in modo process-local il replay engine V3 congelato usando snapshot
   V5 verificati prima del binding;
4. esegue una sola volta il replay seed 125 con policy
   `verified_status0_max_iter_v1`;
5. valuta il candidato senza update e ne verifica l'identità rispetto al
   freeze.

Lo schema interno del replay resta V3 per rendere esplicita la provenance;
wrapper, gate, receipt e ledger sono schema V5.

## Stati e autorità

- freeze: `H0_PRIMARY_GRF_SPLIT_V5_EXECUTION_FROZEN`;
- offline: `PASS/FAIL_H0_PRIMARY_SPLIT_V5_OFFLINE`;
- replay wrapper: `PASS/FAIL_H0_PRIMARY_SPLIT_V5_REPLAY`;
- terminale: `PASS/FAIL_H0_PRIMARY_SPLIT_V5_FINAL_HOLDOUT`.

Il PASS abilita solo `CANONICAL_CLOSED_LOOP_QUALIFICATION`; non promuove il
runtime, non avvia PPO e non apre trial protetti. Ogni FAIL porta a
`STOP_WITHOUT_RETRY_OR_RETUNING`.

## File e stato di preparazione

La lineage comprende esclusivamente nuovi file V5:

- `validation/h0_primary_grf_split_v5_freeze_contract.py`;
- `validation/run_h0_primary_grf_split_v5_full_mean.py`;
- `validation/freeze_h0_primary_grf_split_v5_execution.py`;
- `validation/build_h0_primary_grf_split_v5_preflight_receipt.py`;
- tre moduli test V5 dedicati.

In questa fase si eseguono soltanto pytest, ruff e py_compile. Il builder non
viene eseguito e restano assenti receipt preflight, lock e run root V5.
