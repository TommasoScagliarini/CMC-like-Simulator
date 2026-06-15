# Valutazione rollout training imitativo V3 governato

**Data:** 2026-06-12

## Problema

Al termine del training imitativo V3 con plant nominale `SEA_Knee Kp=18` e
reference governor, era necessario verificare se la policy avesse:

- appreso una corretta imitazione della gamba sana;
- eliminato il comportamento bang-bang e il clamp interno del SEA del
  ginocchio;
- migliorato il contatto protesico e ridotto il supporto delle reserve;
- raggiunto prestazioni confrontabili con l'oracle imitativo governato.

Il rollout deterministico valutato e:

`Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win_rollout`

Il checkpoint utilizzato e:

`runs/baseline_mlp_imit_v3_governed_win/rl_module_best`

## Soluzione e strategia di valutazione

La policy V3 e stata valutata tramite un rollout deterministico completo da
`201` step con registrazione degli output OpenSim. Le metriche sono state
confrontate con:

- la precedente policy imitativa non governata;
- l'oracle finale con `Kp=18` e reference governor;
- i segnali interni del SEA, inclusi `tau_input_raw`, errore di coppia,
  velocita motore e clamp interno;
- contatto, penetrazione, reserve e quota muscolare;
- contenuto spettrale del comando high-level e delle dinamiche interne SEA.

Sono stati inoltre generati i plot diagnostici temporali tramite
`plot/plotter.py`.

## Risultato del rollout

Il rollout e terminato correttamente per normale time limit:

| Metrica | Valore |
|---|---:|
| Step | `201` |
| Episode return | `124.9171` |
| Reward media | `0.6215` |
| Reward minima | `0.1314` |
| Reward massima | `0.8455` |
| Minimo `pelvis_ty` | `0.9449 m` |
| Terminazione anticipata | no |
| Troncamento per time limit | si |
| Massimo output grezzo policy | `1.6768` |

Il rollout deterministico supera il miglior return medio osservato durante il
training, pari a circa `103.09` all'iterazione 39. Il training era ancora in
crescita nelle ultime iterazioni, ma l'entropia finale rimaneva elevata.

## Confronto imitativo

| Metrica | Policy precedente | V3 governata | Oracle governato |
|---|---:|---:|---:|
| Knee imitation RMSE | `24.07 deg` | **`15.55 deg`** | `5.90 deg` |
| Knee imitation MAE | `22.45 deg` | **`9.87 deg`** | `4.22 deg` |
| Knee correlazione | `-0.061` | **`0.608`** | `0.938` |
| Ankle imitation RMSE | `6.62 deg` | **`4.52 deg`** | `2.04 deg` |
| Ankle correlazione | `0.326` | **`0.741`** | `0.935` |

La caviglia raggiunge una buona imitazione. Il ginocchio migliora
significativamente rispetto alla policy precedente, ma non raggiunge ancora
una corretta imitazione:

- la policy sotto-riproduce la flessione del ginocchio;
- la deviazione standard del knee angle e `8.01 deg`, contro `16.29 deg`
  dell'oracle;
- il knee angle medio e `-16.03 deg`, contro `-25.17 deg` dell'oracle.

## Dinamica SEA e saturazioni

| Metrica knee | Policy precedente | V3 governata | Oracle governato |
|---|---:|---:|---:|
| Errore coppia RMS | `41.89 Nm` | **`11.83 Nm`** | `4.20 Nm` |
| Clamp interno `tau_input` | `33.93%` | **`0%`** | `0%` |
| `tau_input_raw` RMS | `587.53 Nm` | **`158.66 Nm`** | `59.14 Nm` |
| `tau_input_raw` massimo | `2794.16 Nm` | **`314.51 Nm`** | `183.61 Nm` |
| Velocita motore RMS | `46.44 rad/s` | **`14.29 rad/s`** | `5.21 rad/s` |
| Velocita motore massima | `137.31 rad/s` | **`32.07 rad/s`** | `14.56 rad/s` |
| Potenza motore RMS | `15930.6 W` | **`1959.35 W`** | `304.62 W` |

Il reference governor ha eliminato il clamp interno pur mantenendo `Kp=18`.
Questo conferma che la causa primaria del precedente railing non era il gain
nominale in se, ma il riferimento high-level discontinuo e fisicamente
incompatibile.

Il comando high-level knee e ora molto piu continuo: la quota di energia sopra
`10 Hz` scende da circa `87.9%` a `5.0%`. Rimane tuttavia un'oscillazione
interna SEA attorno a `100 Hz`, presente anche nell'oracle ma con ampiezza molto
inferiore.

## Contatto e supporto biologico

| Metrica | Policy precedente | V3 governata | Oracle governato |
|---|---:|---:|---:|
| Contatto piede sinistro | `6.90%` | **`71.66%`** | `60.37%` |
| Forza normale sinistra media | `13.46 N` | **`595.97 N`** | `321.76 N` |
| Reserve norm media | `798.62` | **`346.14`** | `244.17` |
| Quota muscolare media | `0.137` | **`0.239`** | `0.267` |

Il contatto protesico e stato ripristinato e le reserve sono piu che dimezzate.
La policy V3 tende pero a caricare eccessivamente il piede protesico. La
penetrazione massima sinistra e `25.2 mm`, inferiore alla terminazione a
`28 mm` ma superiore alla soglia di penalita soft a `20 mm`.

## Diagnosi conclusiva

Il V3 rappresenta un miglioramento sostanziale:

- elimina il clamp interno del ginocchio;
- rimuove il bang-bang dal comando high-level;
- migliora nettamente imitazione, contatto, reserve e potenza motore;
- dimostra che `Kp=18` e utilizzabile con un riferimento governato.

Il risultato non e ancora una corretta imitazione del ginocchio. Il collo di
bottiglia principale non e piu il plugin SEA o la continuita del riferimento,
ma la traiettoria appresa dalla policy:

- il ginocchio sotto-flette rispetto al target;
- il comando SEA knee raggiunge `u=+1` per circa l'`11%` del rollout;
- l'output grezzo della policy arriva a `1.6768` e viene clippato;
- la richiesta sostenuta amplifica l'oscillazione interna SEA;
- il piede protesico viene caricato piu dell'oracle.

Non e quindi consigliato ridurre nuovamente `Kp`. Prima di estendere molto il
training occorre intervenire sulla distribuzione delle azioni e sulla reward,
per ridurre il clipping e premiare maggiormente ampiezza e fase della flessione
del ginocchio senza aumentare penetrazione e carico.

## File modificati e output generati

Durante questa fase di valutazione non sono stati modificati file sorgente.

Output analizzati:

- `Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win_rollout/`
- `Trajectory Generator/runs/baseline_mlp_imit_win_rollout/`
- `Trajectory Generator/runs/imitation_oracle_kp18_governed_final/`

Plot generati:

- `plot/06_12_2026_1/01_time_sea_control_reserve.png`
- `plot/06_12_2026_1/02_time_joint_motor_states.png`
- `plot/06_12_2026_1/03_gaitcycle_torque_angle_power.png`
- `plot/06_12_2026_1/04_gaitcycle_joint_velocity_power.png`
- `plot/06_12_2026_1/05_time_tau_input_tracking_error.png`
- `plot/06_12_2026_1/06_time_joint_ref_sea_error.png`

Il rollout da `2 s` non contiene cicli completi sufficienti per valorizzare
correttamente tutti i plot gait-cycle delle figure 3 e 4. I plot temporali
1, 2, 5 e 6 sono utilizzabili.

## Test e verifiche eseguite

- verifica di `rollout_summary.json` e completamento regolare dell'episodio;
- confronto diretto degli STO tra policy precedente, V3 e oracle;
- calcolo di RMSE, MAE e correlazione del target imitativo phase-based;
- analisi di clamp interno, errore di coppia, `tau_input_raw`, velocita,
  accelerazione e potenza motore;
- analisi del clipping del comando normalizzato SEA;
- analisi spettrale del comando high-level e delle variabili interne SEA;
- confronto di contatto, penetrazione, reserve e quota muscolare;
- generazione verificata dei plot nella cartella root `plot/`.

## Comandi verificati

### Inference con registrazione output

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-rllib python "Trajectory Generator\baseline_MLP\rollout_eval.py" --checkpoint "runs\baseline_mlp_imit_v3_governed_win\rl_module_best" --output-dir "runs\baseline_mlp_imit_v3_governed_win_rollout" --record-outputs
```

### Visualizzazione OpenSim

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-like python visualize.py --sto "Trajectory Generator\runs\baseline_mlp_imit_v3_governed_win_rollout\sim_outputs\rollout_episode_kinematics.sto" --model "models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500_pi.osim" --speed 0.5 --loop
```

### Generazione plot nella cartella `plot/`

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-like python plot\plotter.py --results-dir "Trajectory Generator\runs\baseline_mlp_imit_v3_governed_win_rollout\sim_outputs" --prefix rollout_episode --setup "models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500_pi_setup.xml" --gait-side all --out-root plot
```

## TODO

- [ ] Ridurre o eliminare il clipping dell'output grezzo della policy,
      valutando una distribuzione d'azione bounded/squashed.
- [ ] Migliorare ampiezza e fase della flessione del ginocchio nella reward
      imitativa.
- [ ] Ridurre il carico e la penetrazione del piede protesico senza perdere il
      contatto ottenuto.
- [ ] Valutare l'oscillazione interna SEA attorno a `100 Hz` dopo aver ridotto
      la richiesta sostenuta della policy.
- [ ] Eseguire un rollout piu lungo per validare generalizzazione oltre
      l'orizzonte di training e produrre plot gait-cycle completi.
