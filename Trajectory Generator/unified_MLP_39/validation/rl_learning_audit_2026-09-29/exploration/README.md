# Esplorazione nel singolo batch PPO

Il batch contiene 4096 transizioni fisiche complete distribuite in 14 frammenti:
12 restano aperti al limite della raccolta e due terminano per
`grf_penetration`. Nessun episodio completa qui i 500 step nominali. I 13 worker
non sono 13 repliche indipendenti del training.

`comparison.json` confronta ciascun blocco contiguo con best80 nominale agli
stessi istanti assoluti, verificando target q, qdot e fase prescribed di entrambi
i giunti. Tutte le 4096 transizioni hanno un confronto valido. Non si uniscono
frammenti o interruzioni. Le configurazioni reward coincidono; la ricostruzione
ha errore massimo zero sulle transizioni del batch e 2.84e-13 sulla somma nominale.

Due blocchi mostrano RMSE di posizione inferiore su entrambi i giunti. Uno dura
315 step: ginocchio/caviglia 4.948/2.751°, contro 5.494/3.055° della nominale sullo
stesso intervallo. L'altro dura soltanto 5 step e descrive un breve avvio.
Nessuno dei 14 blocchi migliora contemporaneamente l'errore di forma normalizzata
di entrambi i giunti; nessuno ha somma reward superiore sul proprio intervallo
completo abbinato. Le differenze reward vanno da −0.051 a −67.688.

Nel blocco lungo con RMSE migliore, la differenza reward è −9.209. I contributi
positivi dell'imitazione aumentano: circa +0.616 actual e +2.777 served dopo i
pesi di blending. La penalità interna aumenta di 10.364; penetrazione, momento
caviglia e out-of-bounds aggiungono insieme circa 2.237. Quindi l'obiettivo
riconosce il miglioramento imitativo, ma gli altri costi lo superano. La forma
normalizzata peggiora in questo blocco di circa 0.027 per entrambi i giunti.
Queste osservazioni non determinano il vero vantaggio di una singola azione né
il rendimento dell'episodio futuro non osservato.

## Distribuzione e clipping

`exploration_metrics.json` misura sigma media 0.733/0.794 e clipping nel connector
del 31.98%/34.38% delle componenti ginocchio/caviglia. Il reward wrapper riceve
già l'azione trasformata/clippata da RLlib: i suoi termini di clipping non
rappresentano tutto il clipping della Gaussiana raw.

`gaussian_clipping.json` usa prima e finale sugli stessi 4096 stati del batch,
escludendo le 14 righe artificiali di bootstrap. Per a~N(μ,σ²):

    d E[clip(a,-1,1)] / dμ = P(-1 < a < 1), con σ fissata

La probabilità media interna ai limiti è 0.679/0.660 prima dell'update. La
differenza fra comando angolare atteso e comando della media clippata vale
−3.394°/−3.009° prima e −3.630°/−3.071° dopo, con i segni OpenSim. La conversione
azione→endpoint è stata verificata sui 4096 record: scale 0.75/0.70 rad e residuo
massimo inferiore a 1.2e-14 rad.

Sul medesimo insieme di stati, il comando atteso cambia mediamente di
+0.044°/+0.236°; il comando della media clippata cambia di +0.281°/+0.298°.
Sigma cambia di +0.03369/−0.01010. Questi valori riguardano l'endpoint prima del
governor: non sono sensibilità del giunto fisico, risposta filtrata, né misura
del cambiamento nei nuovi stati visitati dalla policy finale.

## Artefatti e verifiche

- `exploration_metrics.json`, `exploration_records.jsonl`: adapter dei native
  SingleAgentEpisode pre-GAE; join `episode_id, episode_t`.
- `comparison.json`: metriche per frammento/blocco, motivi terminali, reward e
  target verificati, hash degli input.
- `gaussian_clipping.json`, `gaussian_clipping_arrays.npz`: attese analitiche,
  probabilità e derivate per stato e componente, maschera real/bootstrap.
- Figura standalone: `plot/09_29_2026_rl_learning_audit/exploration_reference_comparison.png`.
- Sei test dell'adapter sono passati; controlli sintetici del comparatore hanno
  verificato identità nominale, rigetto target diverso, confini dei frammenti,
  esclusione del segmento incompleto dalla cinematica e conservazione del costo.
- La formula della Gaussiana clippata è stata confrontata con quadratura numerica;
  la derivata rispetto a μ con differenze finite. Nessuna simulazione aggiuntiva.

Comandi (dalla directory superiore a questa):

```sh
/opt/anaconda3/envs/envCMC-rllib/bin/python exploration_analysis.py \
  ../../runs/training/MLP39_LEFT_best80_rl_audit_2026-09-29/ppo_audit/episodes_pre_gae.pkl \
  --output-dir exploration --normalize-actions true --clip-actions false --gamma .99 --lambda .9
/opt/anaconda3/envs/envCMC-rllib/bin/python exploration_reference_comparison.py \
  exploration/exploration_records.jsonl \
  --reward-config ../../runs/training/MLP39_LEFT_best80_rl_audit_2026-09-29/training_cfg.resolved.yaml \
  --output-dir exploration --plot
/opt/anaconda3/envs/envCMC-rllib/bin/python exploration_gaussian_clipping.py \
  learner_analysis/predictions.npz --records exploration/exploration_records.jsonl \
  --output-dir exploration
```

Le figure e i conteggi sono descrittivi: finestre diverse non costituiscono un
ranking di ritorni completi. Il batch non registra qui coppie SEA a 1 kHz;
migliore cinematica parziale non dimostra parità della forma delle coppie con V26.
