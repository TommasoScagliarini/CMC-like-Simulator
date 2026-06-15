# Monitoraggio del critico nel prossimo training: metriche del value function

**Data**: 2026-06-11
**Contesto**: prosecuzione della sessione di tuning del 2026-06-10 sul critico
(explained-variance cronicamente negativa). Vedi
[[2026-06-10_observation_space_realistico_critico_privilegiato]] (critico
privilegiato asimmetrico) e la discussione γ/λ/`vf_clip_param`.

## Problema

Il critico (value function) ha explained-variance ≈ 0 o negativa in modo cronico:
non riesce a predire il return → advantage a varianza alta → gradiente di policy
peggiore (impatta soprattutto gli episodi lunghi). Due cause sospette discusse:
(1) critico "cieco" sullo stato che determina il return (pelvis/GRF/controlaterale)
→ mitigata dal **critico privilegiato asimmetrico** già implementato; (2)
`vf_clip_param` mal-scalato (default RLlib 10 vs target di valore ~70-80 con
γ=0.99) → il value non riesce a inseguire target grandi.

## Azione già applicata

- **γ = 0.95** (era 0.99) e **λ = 0.9** (era 0.95) come nuovi default in
  `train_ppo_mlp.py` (`--gamma`/`--lam` → `PPOConfig.training(gamma, lambda_)`).
  Razionale: la reward imitation è **densa** (segnale per-step), non serve orizzonte
  lungo; γ=0.95 accorcia l'orizzonte effettivo (~100 → ~20 step) e **rimpicciolisce
  i target di valore** (~77 → ~18), dimezzando da solo il problema di scala del clip.
- `vf_clip_param` **lasciato al default 10**, di proposito: prima di toccarlo
  vogliamo i dati delle due metriche sotto.

## Cosa monitorare nel prossimo training

1. **`vf_explained_var`** — se resta **≪ 0 nel tempo** (non solo all'iter 1, dove
   un valore leggermente negativo è fisiologico con policy quasi random), il value
   **non sta fittando**.
2. **Valore medio predetto vs return medio** — se il critico predice
   **sistematicamente molto meno** del return osservato (es. predice ~5 mentre il
   return-to-go medio è ~18), è il segnale che il **clip lo sta strozzando**.

Dove leggerle:
- `runs/<run>/train_iterations.jsonl` → `vf_loss` per iterazione e il blocco
  `learner_metrics` (`learners/default_policy/vf_explained_var`).
- TensorBoard (`runs/<run>/tensorboard`) → `learners/default_policy/vf_explained_var`,
  `vf_loss`; confronto con `env_runners/.../episode_return_mean` (tenendo conto dello
  sconto γ nel passare da return a return-to-go).

## Strategia / azioni conseguenti

- Se **`vf_explained_var` diventa positiva** con γ=0.95 (+ eventuale
  `--critic-privileged-observation`): **non toccare `vf_clip_param`** — non serve.
- Se **resta ≪ 0** e si vede il **divario predetto≪return**: portare
  `vf_clip_param` a **~20-30** (sulla scala dei nuovi target ~18) **oppure
  disattivarlo** (il value-clipping è controverso in letteratura: spesso non aiuta o
  peggiora). `vf_clip_param`/`vf_loss_coeff` oggi NON sono esposti come flag né
  impostati nel `.training()` → all'occorrenza vanno aggiunti (leva pronta, non
  ancora attivata).
- Raccomandato per il prossimo run "vero": **abilitare il critico privilegiato**
  (`--critic-privileged-observation`) — attacca la causa radice della EV negativa,
  ortogonale a γ e a `vf_clip_param`.

## File modificati

```text
Trajectory Generator/baseline_MLP/train_ppo_mlp.py   (default --gamma 0.95, --lam 0.9)
```

## Verifiche eseguite

- `py_compile` implicito (modifica di soli default argparse, nessun cambio logico).
- Nessun run nuovo lanciato per questa modifica: i default valgono dal **prossimo
  avvio**. Il run imitativo in corso (`runs/baseline_mlp_imit_win`) ha letto i
  vecchi default (γ=0.99, λ=0.95) all'avvio e li mantiene; adotterebbe 0.95/0.9 solo
  a un eventuale restart del wrapper auto-restart dopo un crash.

## TODO

- [ ] Nel prossimo training, **monitorare `vf_explained_var` e il divario
      predetto-vs-return** secondo i criteri sopra; decidere su `vf_clip_param`
      (≈20-30 / off) solo in base ai dati.
- [ ] Valutare di **esporre `vf_clip_param` e `vf_loss_coeff` come flag CLI** e
      aggiungerli al `.training()` (leva esplicita, default invariato finché non
      servono).
- [ ] Lanciare un run con **`--critic-privileged-observation`** e confrontare la EV
      del critico contro il baseline symmetric.
