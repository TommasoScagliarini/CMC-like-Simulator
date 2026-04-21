# Tracking CMC-like 100/20 - esito implementazione

Data: 2026-04-20

## Obiettivo

Portare la simulazione a gain esterni SEA fissi `sea_kp=100`, `sea_kd=20`
verso un comportamento CMC-like: full run stabile, tracking protesico stretto,
`tau_input` senza clamp e con bassa energia HPF.

## Modifiche implementate

- Creato il modello dedicato:
  `models/Adjusted_SEASEA - Copia_tuned_inner10020.osim`
  - `SEA_Knee`: `stiffness=250`, `Kp=18.5`, `Kd=10`
  - `SEA_Ankle`: `stiffness=500`, `Kp=35`, `Kd=20`
- Aggiunto gate strict:
  `validation/cmc_strict_metrics.py`
  - verifica run completa, output finiti, validator senza FAIL;
  - verifica tracking protesico RMS/max;
  - verifica HPF di `tau_input`;
  - verifica saturazioni, `max |u|`, `max |tau_input_raw|`;
  - verifica anche `sea_feasibility_scale == 1` per almeno il 95% dei campioni,
    quando la diagnostica di fallback e' presente.
- Aggiunto sweep helper:
  `validation/sea_inner_10020_sweep.py`
  - genera modelli `.osim` copiati dal template;
  - esplora la griglia richiesta di `K`, `omega_n`, `zeta`;
  - deriva `Kp = Jm*omega_n^2/K - 1`;
  - deriva `Kd = 2*zeta*Jm*omega_n - Bm`;
  - scarta candidati non fisici;
  - lancia le finestre screening e scrive CSV.
- Aggiunto fallback SEA CMC-like disattivo di default:
  `enable_sea_feasibility_scaling = False`
  - scala solo la componente PD protesica;
  - lascia invariato il feed-forward;
  - predice `tau_input_raw` e `u`;
  - registra `*_sea_feasibility_scale`.
- Estesa la diagnostica SEA con:
  - `*_outer_pd_unscaled_cmd`
  - `*_sea_feasibility_scale`

Non e' stato modificato il plugin C++.

## Screening eseguito

Finestre principali usate per debug rapido:

- `4.26 -> 5.30 s`
- alcune prove su variante fallback con `--sea-feasibility-scaling`

Soglie strict principali:

- RMS protesico <= `3 deg`
- max protesico <= `10 deg`
- HPF `tau_input` <= `0.05`
- saturazioni `tau_input` = `0`
- `max |u| < 0.95`
- `max |tau_input_raw| < 450 Nm`
- se fallback attivo: `scale == 1` in almeno `95%` dei campioni

## Risultati short-run

| Run | Status | Knee RMS deg | Ankle RMS deg | Knee HPF | Ankle HPF | Sat knee/ankle | Raw max knee/ankle Nm | Max u knee/ankle | Scale==1 knee/ankle |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Baseline inner, no feasibility | failed @ 4.600 | overflow | overflow | 0.790 | 0.917 | 107 / 322 | overflow | 1.000 / 1.000 | n/a |
| Baseline inner + feasibility | complete | 0.389 | 6.904 | 0.894 | 0.944 | 0 / 32 | 414.9 / 748.7 | 0.307 / 0.361 | 0.997 / 0.127 |
| Current inner + feasibility | complete | 2.395 | 17.361 | 0.923 | 0.901 | 0 / 80 | 397.2 / 899.9 | 0.934 / 0.681 | 0.965 / 0.055 |
| `kk1000_ka1250_wn700_z0p7` | complete | 7.582 | 127.541 | 0.976 | 0.959 | 24 / 228 | 880.9 / 1626.5 | 1.000 / 1.000 | 1.000 / 1.000 |
| `kk1000_ka1000_wn1100_z1p2` | complete | 13.254 | 335.815 | 0.939 | 0.957 | 190 / 590 | 2307.0 / 6168.8 | 1.000 / 1.000 | 1.000 / 1.000 |
| `kk1000_ka500_wn900_z0p7` | complete | 430.082 | 698.260 | 0.742 | 0.905 | 94 / 845 | 1676.5 / 10869.8 | 1.000 / 1.000 | 1.000 / 1.000 |
| `kk1000_ka500_wn700_z1p2` | complete | 224.305 | 258.352 | 0.613 | 0.938 | 35 / 413 | 738.0 / 4816.4 | 1.000 / 1.000 | 1.000 / 1.000 |

## Diagnosi

La baseline inner storica e' il punto piu vicino al tracking desiderato:

- knee passa bene;
- ankle resta fuori soglia;
- `tau_input` resta fortemente ad alta frequenza;
- la fattibilita' PD salva la run ma sulla caviglia interviene per quasi tutta
  la finestra (`scale==1` solo nel 12.7% dei campioni).

Questo viola esplicitamente il criterio del piano: il fallback non puo' essere
accettato se diventa il comportamento dominante.

I candidati inner piu rigidi o piu veloci non riducono il problema: con gain
esterni `100/20` saturano `u` e `tau_input`, peggiorano il tracking protesico e
trascinano pelvis/biologico. I candidati piu smorzati riducono parzialmente il
raw knee, ma degradano comunque tracking e caviglia.

Conclusione operativa: con i vincoli attuali, la sola modifica fisica di
`stiffness/Kp/Kd` nel modello non ha prodotto un candidato che passi lo
screening strict. Per questo non ho aggiornato `config.py` al modello
`inner10020` e non ho lanciato una full acceptance su candidati gia' bocciati
dalle finestre brevi.

## Verifica codice

Compilazione eseguita con successo:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile config.py simulation_runner.py static_optimization.py output.py inverse_dynamics.py prosthesis_controller.py main.py validation/cmc_strict_metrics.py validation/sea_inner_10020_sweep.py
```

## File modificati o creati

- `config.py`
- `main.py`
- `prosthesis_controller.py`
- `output.py`
- `validation/cmc_strict_metrics.py`
- `validation/sea_inner_10020_sweep.py`
- `models/Adjusted_SEASEA - Copia_tuned_inner10020.osim`
- `models/Adjusted_SEASEA - Copia_tuned_kk1000_ka1000_wn1100_z1p2.osim`
- `models/Adjusted_SEASEA - Copia_tuned_kk1000_ka500_wn900_z0p7.osim`
- `models/Adjusted_SEASEA - Copia_tuned_kk1000_ka500_wn700_z1p2.osim`
- `reports/user/2026-04-20_tracking_cmc_like_100_20.md`

## Stato finale

Il codice ora misura e boccia correttamente i casi non CMC-like:

- non accetta saturazioni mascherate;
- non accetta HPF alto su `tau_input`;
- non accetta il fallback se la scala SEA diventa sistematica;
- conserva il vincolo `sea_kp/sea_kd = 100/20`.

La simulazione stabile a `100/20` esiste, ma non e' ancora cinematicamente e
dinamicamente CMC-like secondo le soglie strict. La causa residua sembra essere
la compatibilita' tra richiesta del PD esterno `100/20` e fattibilita' fisica
della caviglia SEA, non un crash numerico isolato.
