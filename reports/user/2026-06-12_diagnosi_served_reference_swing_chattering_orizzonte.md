# Diagnosi served reference, swing iniziale, chattering e orizzonte

**Data:** 2026-06-12

## Problema

Dopo la prima valutazione del rollout imitativo V3, l'ispezione visiva del
video e dei plot ha evidenziato apparenti incongruenze:

- un grande swing del ginocchio all'inizio dell'episodio;
- una caviglia apparentemente immobile nel video;
- una forte somiglianza tra la curva indicata come `kinematic ref` nel plot 06
  e la cinematica prescribed;
- assenza apparente dello swing iniziale nel plot 06;
- scarso inseguimento apparente del riferimento generato;
- forte chattering nei segnali di coppia SEA, soprattutto al ginocchio;
- episodio troppo corto per osservare comodamente un ciclo protesico completo.

Era necessario distinguere tra:

1. errore della policy nel generare la traiettoria;
2. errore di tracking del controllore high-level;
3. dinamica interna del SEA;
4. errore diagnostico nei plot.

Il controllore e il plugin SEA non sono stati assunti come causa iniziale,
poiche erano gia stati validati con riferimenti prescribed.

## Errore individuato nel plot 06

Il precedente:

`plot/06_12_2026_1/06_time_joint_ref_sea_error.png`

non confrontava la cinematica simulata con il riferimento effettivamente
generato e servito dalla policy.

La curva indicata come `kinematic ref` veniva caricata direttamente dal file IK
prescribed configurato nel setup:

`models/AB06_SEASEA_Threadmill/data/IK_results_AB06_SEASEA.mot`

Questo spiegava contemporaneamente:

- la somiglianza della curva con i precedenti dati prescribed;
- l'assenza dello swing iniziale visto nel video;
- l'apparente scarso inseguimento del riferimento generato.

Il plot 06 non poteva quindi essere usato per valutare il tracking della
traiettoria prodotta dalla rete.

## Soluzione diagnostica

E stata aggiunta la registrazione esplicita del riferimento realmente servito
al controllore durante ogni substep:

- `q_ref`;
- `qdot_ref`.

Il nuovo output e:

`rollout_episode_kinematics_reference.sto`

Il plotter ora usa questo file quando disponibile. L'IK prescribed viene
utilizzato solamente come fallback per output storici che non contengono il
riferimento servito.

Il rollout diagnostico equivalente al V3 originale e:

`Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win_rollout_refdiag`

Il nuovo plot corretto e:

`plot/06_12_2026_2/06_time_joint_ref_sea_error.png`

Il rollout diagnostico riproduce esattamente il precedente episode return
`124.9171`, confermando che la nuova registrazione non modifica la simulazione.

## Tracking reale del riferimento generato

Confrontando la cinematica simulata con il riferimento realmente servito:

| Errore `simulated - served reference` | Knee | Ankle |
|---|---:|---:|
| RMSE globale | **`2.50 deg`** | **`0.55 deg`** |
| Errore massimo | `5.33 deg` | `1.97 deg` |
| RMSE primi `250 ms` | `0.36 deg` | `0.91 deg` |

Il controllore high-level insegue quindi correttamente il riferimento prodotto
dalla policy. Il mancato tracking ipotizzato dalla prima lettura del plot era
principalmente un artefatto diagnostico.

La traiettoria realmente servita rimane pero diversa dal prescribed:

| Confronto | Knee RMSE | Ankle RMSE |
|---|---:|---:|
| Served reference vs prescribed | `17.50 deg` | `10.28 deg` |
| Simulated vs prescribed | `17.28 deg` | `10.08 deg` |

Il collo di bottiglia principale e quindi la traiettoria scelta dalla policy,
non la capacita del controllore di inseguirla.

Le precedenti metriche di imitazione rispetto al target sound-leg phase-based
restano valide; viene corretta solamente l'interpretazione del plot 06 rispetto
al riferimento policy.

## Causa dello swing iniziale

Lo swing iniziale del ginocchio e presente sia nel video sia nei dati
registrati.

Valori principali:

| Istante | Knee simulated | Knee served reference | Knee prescribed |
|---|---:|---:|---:|
| Inizio episodio | `-53.48 deg` | `-53.30 deg` | `-7.95 deg` |
| Dopo `100 ms` | `-29.54 deg` | `-29.71 deg` | `-7.83 deg` |
| Dopo `250 ms` | `-6.10 deg` | `-6.72 deg` | `-7.42 deg` |

L'episodio imitativo viene inizializzato coerentemente sul target phase-based,
che in quell'istante richiede circa `-53 deg` di flessione del ginocchio.

Il primo segmento emesso dalla policy contiene invece un primo knot knee
clippato a circa `0 deg`. La policy chiede quindi immediatamente una grande
estensione del ginocchio. Il reference governor limita velocita e accelerazione,
ma non elimina lo spostamento richiesto.

Il tracking durante questo transitorio e buono: nei primi `250 ms` l'errore
knee rimane entro circa `0.67 deg`. Lo swing e quindi generato dal riferimento
policy, non da un errore del controllore.

E stata inoltre aggiunta una traccia step-level:

`rollout_policy_trace.json`

contenente:

- output grezzo della policy;
- knot convertiti e clippati;
- tempi, valori e derivate del segmento;
- target imitativo disponibile durante il rollout.

## Movimento della caviglia

La caviglia non e immobile numericamente:

- served reference: da `3.04 deg` a `24.23 deg`;
- simulated joint angle: da `2.93 deg` a `24.64 deg`;
- escursione complessiva simulata: circa `21.7 deg`;
- tracking RMSE: `0.55 deg`.

Il movimento e meno evidente nel video perche ha un'escursione visiva minore
del ginocchio ed e concentrato prevalentemente durante il contatto.

## Causa del chattering SEA

Il riferimento di posizione e velocita servito e continuo. Il chattering non
deriva quindi direttamente da rumore ad alta frequenza su `q_ref`.

La causa identificata e l'interazione tra:

1. aggiornamento della policy ogni `10 ms`, equivalente a `100 Hz`;
2. continuita di posizione e velocita tra segmenti, ma discontinuita di
   accelerazione ai confini dei segmenti;
3. dinamica interna del SEA knee vicina alla stessa banda;
4. richiesta sostenuta e molto maggiore imposta dalla traiettoria policy;
5. feedback cascade di velocita, che converte il ripple articolare in richiesta
   di coppia.

Nel riferimento knee derivato compare una componente dominante di accelerazione
a circa `99.95 Hz`. I salti di accelerazione raggiungono il limite del governor
di `60 rad/s^2`.

La frequenza naturale linearizzata del torque loop SEA knee e:

```text
fn = sqrt((1 + Kp) * K / Jm) / (2*pi)
   = sqrt(19 * 321 / 0.01) / (2*pi)
   ~= 124 Hz
```

Il forcing introdotto dall'aggiornamento a `100 Hz` cade quindi vicino alla
banda della dinamica interna SEA.

### Confronto corretto con prescribed in modalita plugin

Il confronto e stato effettuato con:

`results/online_grf_gait2s_prescribed`

Questo run usa `sea_forward_mode=plugin`; non e stato usato il vecchio baseline
`ideal_torque`, che bypassa proprio la dinamica SEA da analizzare.

| Metrica knee | Prescribed plugin | Policy V3 |
|---|---:|---:|
| `tau_ref` RMS | `11.68 Nm` | `55.62 Nm` |
| Torque error RMS | `1.30 Nm` | `11.83 Nm` |
| `tau_input_raw` RMS | `14.32 Nm` | `158.66 Nm` |
| Componente `tau_ref` a circa `100 Hz` | `0.03 Nm` | `11.45 Nm` |
| Componente joint velocity a circa `100 Hz` | `0.0007 rad/s` | `0.309 rad/s` |

Il controllore e il plugin funzionano correttamente col prescribed perche quel
riferimento non eccita fortemente la banda a `100 Hz` e richiede coppie knee
molto inferiori.

Il V3 non presenta clamp interno, ma il reference segmentato e l'operating
point imposto dalla policy amplificano la risonanza interna.

## Verifica dell'orizzonte piu lungo

E stato eseguito un rollout diagnostico con:

```powershell
--episode-duration 5.0
```

Output:

`Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win_rollout_5s_refdiag`

Il rollout non raggiunge i `5 s`: termina dopo `0.68 s` per
`grf_penetration`, con penetrazione sinistra massima `28.1 mm`.

Il semplice aumento dell'orizzonte cambia infatti una feature osservata dalla
policy:

```python
phase = elapsed_time / episode_duration
```

Passando da `2 s` a `5 s`, il clock dell'episodio evolve a una velocita diversa
e il checkpoint entra immediatamente fuori distribuzione.

Nel rollout originale da `2 s`, il contatto protesico online osservato e:

- heel strike sinistro: `12.100 s`;
- toe-off sinistro: `13.505 s`;
- contatto sinistro successivo: circa `13.965 s`;
- fine episodio: `13.990 s`.

Il rollout contiene quindi quasi un ciclo protesico completo, ma termina subito
dopo il contatto successivo e non offre margine sufficiente per una valutazione
gait-cycle robusta.

Per aumentare correttamente l'orizzonte occorre addestrare nuovamente la policy
con episodi piu lunghi e rendere il clock indipendente dalla durata totale
dell'episodio, usando principalmente il gait-phase clock ciclico gia
disponibile.

## File modificati

- `output.py`
  - registrazione del riferimento realmente servito;
  - nuovo output `*_kinematics_reference.sto`.

- `plot/plotter.py`
  - il plot 06 usa il served reference registrato;
  - il prescribed resta fallback per output storici.

- `Trajectory Generator/baseline_MLP/rollout_eval.py`
  - nuovo `rollout_policy_trace.json`;
  - registrazione di azione grezza, segmenti policy e target imitativo.

Non sono stati modificati:

- controllore high-level SEA;
- plugin C++;
- modello `.osim`;
- reward o configurazione del training.

## Output generati

- `Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win_rollout_refdiag`
- `Trajectory Generator/runs/baseline_mlp_imit_v3_governed_win_rollout_5s_refdiag`
- `plot/06_12_2026_2`

## Test e verifiche eseguite

- compilazione Python con `py_compile` dei file modificati;
- `git diff --check`;
- nuovo rollout V3 da `2 s` con served reference registrato;
- verifica di equivalenza del return rispetto al rollout precedente;
- generazione e ispezione del plot 06 corretto;
- confronto quantitativo `simulated`, `served reference` e prescribed;
- analisi separata dei primi `250 ms`;
- analisi dei primi knot emessi dalla policy;
- confronto spettrale SEA tra V3, oracle e prescribed plugin;
- calcolo della frequenza naturale linearizzata del torque loop SEA;
- rollout di generalizzazione da `5 s`;
- verifica della causa di terminazione `grf_penetration`.

## Comandi verificati

### Rollout diagnostico con served reference

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-rllib python "Trajectory Generator\baseline_MLP\rollout_eval.py" --checkpoint "runs\baseline_mlp_imit_v3_governed_win\rl_module_best" --output-dir "runs\baseline_mlp_imit_v3_governed_win_rollout_refdiag" --record-outputs
```

### Plot corretto

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-like python plot\plotter.py --results-dir "Trajectory Generator\runs\baseline_mlp_imit_v3_governed_win_rollout_refdiag\sim_outputs" --prefix rollout_episode --setup "models\AB06_SEASEA_Threadmill\AB06_SEASEA_stiff321_500_pi_setup.xml" --gait-side all --out-root plot
```

### Test diagnostico a 5 secondi

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-rllib python "Trajectory Generator\baseline_MLP\rollout_eval.py" --checkpoint "runs\baseline_mlp_imit_v3_governed_win\rl_module_best" --output-dir "runs\baseline_mlp_imit_v3_governed_win_rollout_5s_refdiag" --episode-duration 5.0 --record-outputs
```

## TODO

- [ ] Rendere il clock actor indipendente dalla durata totale dell'episodio,
      privilegiando il gait-phase clock ciclico.
- [ ] Addestrare nuovamente con episodi da almeno `3.5-5 s`.
- [ ] Penalizzare la distanza tra primo knot policy e riferimento iniziale per
      eliminare lo swing iniziale.
- [ ] Rendere il reference governor continuo anche in accelerazione,
      preferibilmente con un profilo jerk-limited.
- [ ] Ridurre l'eccitazione a `100 Hz` introdotta dai segmenti policy senza
      modificare il plugin o nascondere il problema abbassando `Kp`.
- [ ] Ridurre o eliminare il clipping dell'output grezzo della policy,
      valutando una distribuzione d'azione bounded/squashed.
- [ ] Migliorare ampiezza e fase della flessione del ginocchio rispetto al
      target imitativo.
- [ ] Ridurre carico e penetrazione del piede protesico senza perdere il
      contatto ottenuto.
- [x] Eseguire un rollout piu lungo per valutare la generalizzazione oltre
      l'orizzonte di training.
- [ ] Ripetere il rollout lungo dopo il nuovo training e produrre plot
      gait-cycle completi.
