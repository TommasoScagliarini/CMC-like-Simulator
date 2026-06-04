# Checkpoint inference path per Prosthesis SNN

Data: 2026-06-01

## Problema

Dopo l'introduzione del wrapper skrl actor-critic condiviso, mancava un ponte
pulito tra training PPO e inference nel simulatore.

Il modello di training contiene componenti non necessari, e non desiderabili,
durante l'inference:

- value head;
- `log_std_parameter` PPO;
- optimizer;
- stato e bookkeeping skrl;
- eventuali stati temporanei del training.

Il simulatore invece deve poter caricare solo la rete di riferimento
protesico, cioe' la parte compatibile con `ProsthesisReferenceSNN` e
`ReferenceGenerator`.

## Soluzione

E' stato aggiunto un export inference-only in:

```text
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/checkpoint.py
```

con due funzioni pubbliche:

```python
build_reference_checkpoint(...)
save_reference_checkpoint(...)
```

Il checkpoint esportato contiene:

- `format`;
- `format_version`;
- `model_state_dict` della sola rete reference;
- `config`;
- `output_transform`;
- `output_scale`;
- `output_offset`;
- `metadata`.

Il checkpoint esclude intenzionalmente:

- value head;
- `log_std_parameter`;
- optimizer;
- stati PPO/skrl.

Il file prodotto e' caricabile direttamente con:

```python
ReferenceGenerator.from_checkpoint(...)
```

## Strategia

La funzione accetta sia:

- `ProsthesisSNNActorCritic`, esportando solo `reference_model`;
- `ProsthesisReferenceSNN`, esportandolo direttamente.

Questo mantiene separati:

- training: actor-critic, PPO, value, log-probability;
- inference: generazione di `q_ref`, `qdot_ref`, `qddot_ref` per protesi.

Il formato del checkpoint resta compatibile con il loader gia' presente in
`ReferenceGenerator`.

## File modificati

```text
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/checkpoint.py
Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/__init__.py
Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py
Trajectory Generator/Prosthesis_SNN/README.md
Trajectory Generator/Prosthesis_SNN/docs/TODO_integration.md
```

## Test e verifiche

Compilazione:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile \
  'Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/checkpoint.py' \
  'Trajectory Generator/Prosthesis_SNN/prosthesis_snn/training/__init__.py' \
  'Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py'
```

Smoke test:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python \
  'Trajectory Generator/Prosthesis_SNN/tests/smoke_test.py'
```

Esito:

```text
smoke tests passed
```

Lo smoke test verifica il percorso:

```text
ProsthesisSNNActorCritic
-> save_reference_checkpoint(...)
-> ReferenceGenerator.from_checkpoint(...)
-> predict(...)
-> riferimenti finiti
```

Controlla anche che il checkpoint inference-only non includa:

- `value_lif.fc.weight`;
- `log_std_parameter`.

## Stato finale

Il checkpoint inference path e' implementato e verificato a livello smoke.

Resta da collegarlo al futuro training entrypoint, cosi' che a fine training
venga esportato automaticamente un checkpoint pulito per inference nel
simulatore CMC-like.
