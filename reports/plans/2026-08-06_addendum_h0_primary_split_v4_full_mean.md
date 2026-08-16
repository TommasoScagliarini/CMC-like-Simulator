# Addendum H0 primary split V4 — adattamento full-mean one-shot

## Decisione e lineage

V3 resta chiuso con `FAIL_H0_PRIMARY_SPLIT_V3_OFFLINE`: il candidato limitato
alle sole colonne 10/11 ha ottenuto RMSE student `0.0105039088860462` e RMSE
teacher `0.007882278226384947`, oltre le soglie rispettive `0.01` e `0.005`.
Il seed 125 non è stato aperto, il candidato V3 non è stato congelato e non è
promuovibile.

V4 è un protocollo nuovo, no-clobber e senza retry. Eredita esclusivamente:

- H0 sorgente immutato;
- corpus V3 PASS costruito dai replay semantici dei seed 123 e 124;
- policy numerica `verified_status0_max_iter_v1` e replay engine V3 congelato;
- ledger e gate V3 come lineage terminale di fallimento.

Il candidato fallito V3 non è un input di training. Non si modificano GRF
primaria, semantica SEA, detector, reward, critic o trainer PPO.

## Candidato unico

L'unico design V4 adatta tutta la rete delle medie dell'actor H0, passando
`trainable_first_layer_features=None` al fitter. La testa log-standard-deviation
resta congelata bit-exact. Non sono ammessi sweep, early stopping, scelta di
checkpoint o secondo candidato.

Parametri chiusi:

- 400 epoche, checkpoint finale fisso;
- seed 123;
- batch 128;
- learning rate `5e-5`;
- anchor `1e-2`;
- validation fraction e patience pari a zero;
- un solo actor update candidate, zero critic update e zero PPO update.

La verifica train-only sui seed 123/124 che ha selezionato questo design ha
prodotto RMSE/max student `0.0030197/0.02477` e RMSE/max teacher
`0.0030618/0.02053`. Questi numeri sono evidenza di design, non sostituiscono
il holdout e non autorizzano altre varianti.

## Gate offline e audit del restore

Il runner verifica nuovamente identità e hash del corpus V3 prima del fit. Il
candidato passa soltanto se:

- student RMSE `<= 0.01`, max error `<= 0.10` e riduzione RMSE `>= 50%`;
- teacher RMSE `<= 0.005` e max error `<= 0.05`;
- logits finiti e medie entro `[-1, 1]`;
- modalità `fixed_final_epoch`, 400 epoche e zero righe di validation;
- modifiche confinate alla rete delle medie, con hidden e mean-output davvero
  modificati;
- parametri e output logstd bit-exact rispetto a H0;
- non-actor esatto o assente nel checkpoint inference-only;
- save/reload dell'actor esatto.

Un FAIL offline chiude V4 senza freeze del candidato e senza accesso al seed
125.

## Holdout procedurale 125

Il freezer registra solo SHA-256 e dimensione dei file storici del seed 125:
non li carica né li interpreta. Dopo il PASS offline il runner:

1. pubblica atomicamente `candidate_freeze.json`;
2. pubblica `holdout_access_claim.json`;
3. invoca il replay engine V3 congelato tramite un wrapper V4 esplicito;
4. esegue il replay con policy SO `verified_status0_max_iter_v1`;
5. valuta il candidato senza update sugli stessi gate student/teacher;
6. verifica che il modulo sia byte-identico al record congelato.

Gli artefatti interni del motore mantengono lo status/schema V3 per rendere
esplicita la provenienza del replay; il wrapper e il gate terminale usano
status V4 e output esclusivamente sotto il run root V4.

## Stati terminali

- PASS offline: `PASS_H0_PRIMARY_SPLIT_V4_OFFLINE`;
- FAIL offline: `FAIL_H0_PRIMARY_SPLIT_V4_OFFLINE`;
- PASS holdout: `PASS_H0_PRIMARY_SPLIT_V4_FINAL_HOLDOUT`;
- FAIL holdout: `FAIL_H0_PRIMARY_SPLIT_V4_FINAL_HOLDOUT`.

Il PASS terminale abilita soltanto `CANONICAL_CLOSED_LOOP_QUALIFICATION`.
Non certifica ancora `CORRIDOR_TRAINING_READY`, non promuove il runtime e non
apre trial protetti. Ogni FAIL porta a `STOP_WITHOUT_RETRY_OR_RETUNING`.

## File del protocollo

- `validation/h0_primary_grf_split_v4_freeze_contract.py`;
- `validation/freeze_h0_primary_grf_split_v4_execution.py`;
- `validation/build_h0_primary_grf_split_v4_preflight_receipt.py`;
- `validation/run_h0_primary_grf_split_v4_full_mean.py`;
- i tre moduli test V4 dedicati.

Prima del freeze, il builder esegue py_compile, ruff e gli 11 test non
ricorsivi e pubblica un receipt no-clobber con hash delle sorgenti testate. Il
dodicesimo test ricostruisce il payload del lock e viene eseguito subito dopo
la pubblicazione del receipt; il freezer rifiuta receipt mancanti o stale.

Il freezer può creare soltanto
`validation/h0_primary_grf_split_v4_execution_lock.json`; il lock e il run non
vengono eseguiti durante la preparazione di questo addendum.
