# Daily Report - 2026-07-08

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `reports/user/2026-07-08_h1_h2_limiter_reward_validation.md`
- `reports/user/2026-07-08_contact_load_penetration_gate_reward.md`

## Problema

Il limiter rendeva il training tecnicamente piu' stabile, ma H2 restava in
stance compressiva: il carico aumentava insieme alla penetrazione e
`contact_load_score` continuava a pagare supporto fino al guard terminale.

## Strategia e soluzione

Prima e' stato ripetuto il gate H1/H2 con limiter. Poi la reward di carico e'
stata moltiplicata per una qualita' di penetrazione:

```text
<= 10 mm : premio pieno
10-12 mm : taper lineare
>= 12 mm : premio nullo
```

La fisica, il plugin SEA e il guard terminale non sono stati alterati.

## Risultati H1/H2 con limiter

- H1 10 iter: best return medio `+1.826`, PASS tecnico;
- H2: `45` step, return `+3.727`, un HS ma nessun TO sinistro;
- termine ancora `grf_penetration`, FAIL comportamentale.

## Risultati dopo il gate del premio di contatto

- H1 10 iter: best return medio `+0.141`, lunghezza `31.5 -> 33.4` step;
- H2: `36` step, return `+0.819`, nessun TO sinistro;
- sopra `12 mm`, `contact_load_score=0` anche con carico crescente;
- il loophole reward e' chiuso, ma la policy continua a comprimere il contatto.

Training 20-50/100 iterazioni restano bloccati.

## File modificati

- `Trajectory Generator/baseline_MLP/reward_function.py`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
- `validation/test_reward_function.py`
- `reports/plans/2026-07-02_piano_validazione_reward_exnovo.md`

## Test e verifiche

- due training H1 da 10 iterazioni: PASS tecnico;
- due rollout H2 registrati: PASS tecnico, FAIL comportamentale;
- gate penetrazione reward verificato riga per riga nel trace;
- reward tests `27/27`, config smoke, `py_compile`, diff check: PASS.

## TODO aperti e propagati

- [ ] Premiare supporto completato/TO senza pagare indefinitamente la
      compressione early-stance.
- [ ] Recuperare il credito provvisorio se stance fallisce prima del TO.
- [ ] Ripetere H1/H2 e richiedere almeno un TO sinistro valido.
- [ ] Non aumentare il budget finche' il gate comportamentale non passa.

