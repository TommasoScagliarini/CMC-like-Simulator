# Daily Report - 2026-07-09

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidato

- `reports/user/2026-07-09_warm_start_imitativo_exnovo.md`

## Problema

Serviva inizializzare la policy ex-novo dalla policy imitativa
`asym100_GRFpenalty-lowered` senza trascinare critic, optimizer e stato PPO del
vecchio objective.

## Soluzione e strategia

E' stato implementato `--warm-start` come transplant actor-only:

- layer actor condivisi copiati per nome feature;
- colonne target-only ex-novo inizializzate a zero;
- critic, optimizer e PPO state freschi;
- nessuna riapplicazione del transplant durante restart interni;
- incompatibilita' esplicita con full resume.

Il trainer esporta:

```text
actor_transplant_report.json
rl_module_initial_warm_start/
```

Questo consente di separare il comportamento introdotto dall'imitazione da
quello appreso successivamente con PPO ex-novo.

## File modificati

- `Trajectory Generator/baseline_MLP/warm_start.py`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`

## Test e verifiche

- `py_compile`: PASS;
- transplant sintetico `31 -> 39` feature actor: PASS;
- shape del checkpoint reale: verificate;
- critic target lasciato invariato: PASS;
- smoke una iterazione: `warm_start_applied=true`, PASS;
- batch reale con un solo env: transplant/esportazione PASS, sampling in
  timeout; non classificato come errore del transplant.

## TODO aperti e propagati

- [ ] Validare `rl_module_initial_warm_start` prima di qualsiasi update PPO.
- [ ] Eseguire H1 warm-start a parita' di config, seed e budget con H1 fresh.
- [ ] Eseguire H2 sul best warm-start e confrontare FSM, penetrazione, clipping,
      reward terms e SEA.
- [ ] Verificare negative transfer e sample efficiency.
- [ ] Risolvere e congelare prima la reward early-stance/contact support.
- [ ] Mantenere bloccati training lunghi e memoria finche' warm start e reward
      non superano il gate comportamentale.

## TODO storico SEA propagato

- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta;
      il punto progettuale del 13/06 non risulta ancora formalmente chiuso.
