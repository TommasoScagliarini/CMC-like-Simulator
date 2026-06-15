# Training imitativo con Kp=18 e reference governor

**Data:** 2026-06-12

## Problema

Il primo training imitativo PPO/MLP produceva una scarsa imitazione del
ginocchio e saturava frequentemente il clamp interno del motore SEA.

Inizialmente era stato valutato di ridurre il gain low-level del ginocchio da
`Kp=18` a `Kp=10`. Questa modifica riduceva le saturazioni, ma alterava un
parametro validato sui dati sperimentali e riduceva la banda del SEA.

Il confronto successivo ha chiarito che `Kp=18` non e la causa primaria:
amplifica gli errori transitori tra `tau_ref` e `tau_spring`, ma con un
riferimento continuo e sperimentalmente compatibile segue correttamente senza
saturare.

L'obiettivo finale e quindi:

- mantenere il plant nominale con `Kp=18`;
- impedire alla policy di generare riferimenti cinematici non fisici;
- conservare un training imitativo capace di apprendere una traiettoria valida,
  senza nascondere i problemi high-level riducendo la banda del SEA.

## Soluzione

### Target imitativo corretto

Il vecchio target basato su uno shift temporale globale e stato sostituito da un
target periodico phase-based, costruito sui cicli locali completi della gamba
sana.

Gli shift calibrati separatamente sono:

| Giunto | Phase shift |
|---|---:|
| Knee | `0.465` |
| Ankle | `0.452` |

Posizione e velocita target derivano dalla stessa spline periodica. In modalita
imitativa, lo stato iniziale viene inoltre installato coerentemente sul target,
eliminando il precedente transitorio iniziale inevitabile.

### Reference governor con Kp=18

Il modello nominale usa nuovamente:

```text
SEA_Knee Kp = 18
```

La policy continua a emettere knot di posizione assoluti, ma il riferimento
effettivamente servito al controllore passa attraverso:

1. un modello di riferimento low-pass del secondo ordine a `6 Hz`;
2. stato di posizione e velocita persistente tra segmenti;
3. limiti duri di velocita;
4. limiti duri di accelerazione;
5. integrazione trapezoidale coerente tra posizione e velocita.

I limiti sono stati scelti con un piccolo margine rispetto ai massimi della
cinematica sperimentale filtrata:

| Giunto | Limite velocita | Limite accelerazione |
|---|---:|---:|
| Knee | `6.0 rad/s` | `60 rad/s^2` |
| Ankle | `3.5 rad/s` | `55 rad/s^2` |

Il governor non modifica la semantica del plugin C++ e non cambia il comando
normalizzato SEA. Agisce solamente sul riferimento cinematico prodotto dalla
policy, dentro `Trajectory Generator/`.

### Reward e diagnostica SEA

La reward imitativa ora misura e penalizza:

- command-rate tra knot e tra step;
- velocita e accelerazione del riferimento servito;
- variazione del comando SEA `u`;
- intervento del reference governor;
- vera saturazione interna `tau_input`;
- errore di coppia `tau_ref - tau_spring`;
- velocita, accelerazione e potenza motore.

Il reference governor e penalizzato solo quando intervengono realmente i limiti
duri. Il normale ritardo introdotto dal low-pass non viene considerato una
violazione fisica.

La diagnostica SEA viene raccolta su ogni substep di integrazione e resa
disponibile alla reward e a TensorBoard separatamente per ginocchio e caviglia.

## Strategia

La modifica e stata validata in ordine:

1. analisi del riferimento sperimentale filtrato per ricavare limiti fisici;
2. test isolato del reference governor su un gradino volutamente irrealistico;
3. confronto oracle tra `Kp=10`, `Kp=18` non governato e `Kp=18` governato;
4. verifica della configurazione salvata e ricostruita da training/rollout;
5. preparazione di un nuovo comando di training da zero.

L'oracle usa lo stesso action mapping, reference model, controllore, plugin SEA
e dinamica ibrida usati dal training, senza dipendere da Torch/RLlib.

## Risultati

### Confronto del tracking SEA del ginocchio

Metriche globali calcolate direttamente da `oracle_episode_sea_diagnostics.sto`:

| Metrica | Kp=18 non governato | Kp=10 precedente | Kp=18 governato |
|---|---:|---:|---:|
| Errore coppia RMS | `12.95 Nm` | `11.59 Nm` | **`4.20 Nm`** |
| Errore coppia medio assoluto | `8.10 Nm` | `7.49 Nm` | **`3.15 Nm`** |
| Errore coppia massimo | `72.97 Nm` | `83.40 Nm` | **`12.58 Nm`** |
| Clamp interno | `4.70%` | `0.65%` | **`0%`** |
| `tau_input_raw` RMS | `202.93 Nm` | `108.19 Nm` | **`59.14 Nm`** |
| `tau_input_raw` massimo | oltre clamp | circa `590 Nm` | **`183.61 Nm`** |
| Velocita motore RMS | `14.96 rad/s` | `8.59 rad/s` | **`5.21 rad/s`** |
| Velocita motore massima | - | - | **`14.56 rad/s`** |

Il risultato dimostra che `Kp=18` e compatibile con il training imitativo quando
il riferimento high-level non contiene transienti irrealistici.

### Oracle finale

Output:

`Trajectory Generator/runs/imitation_oracle_kp18_governed_final`

Metriche principali:

- completamento regolare: `201` step, chiusura per normale time limit;
- reward media: `0.8432`;
- tracking score medio: `0.9900`;
- sound imitation loss medio: `0.03490`;
- clamp interno knee: `0%`;
- clamp interno ankle: `0%`;
- torque error knee medio per segmento: `3.39 Nm RMS`;
- torque error ankle medio per segmento: `0.49 Nm RMS`;
- velocity limiter: `0%` dei campioni;
- acceleration limiter: `5.04%` dei campioni;
- massimo riferimento knee osservato: `1.42 rad/s`, `38.23 rad/s^2`;
- massimo riferimento ankle osservato: `0.53 rad/s`, `27.08 rad/s^2`.

Il governor lascia quindi passare quasi interamente il target imitativo valido e
interviene solamente sui transienti incompatibili.

## File modificati

### Riferimento, target e ambiente

- `Trajectory Generator/osim_trj_cmc_like.py`
  - target imitativo phase-based;
  - inizializzazione imitativa coerente;
  - reference governor;
  - diagnostica command-rate e intervento dei limiti.

- `simulation_runner.py`
  - raccolta diagnostica SEA reale su ogni substep.

### Reward e logging

- `Trajectory Generator/baseline_MLP/reward_function.py`
  - nuove penalita command-rate e stress SEA.

- `Trajectory Generator/baseline_MLP/reward_imitation.json`
  - pesi specifici del training imitativo.

- `Trajectory Generator/baseline_MLP/tb_logging.py`
  - logging delle nuove metriche SEA e reference governor.

- `Trajectory Generator/baseline_MLP/env_factory.py`
  - inizializzazione sul target quando la reward e imitativa.

### Training, rollout e oracle

- `Trajectory Generator/baseline_MLP/imitation_oracle_rollout.py`
  - oracle imitativo indipendente da RLlib.

- `Trajectory Generator/baseline_MLP/training_config.py`
- `Trajectory Generator/baseline_MLP/training_cfg.yaml`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
  - configurazione, snapshot e ricostruzione dei limiti del governor.

- `Trajectory Generator/baseline_MLP/commands.txt`
  - comandi `TRAIN-IMIT-V3-GOV-WIN`, `ORACLE-IMIT-KP18-GOV` e
    `ROLLOUT-IMIT-V3-GOV`.

### Modello SEA

- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim`
  - verificato e ripristinato il valore nominale `SEA_Knee Kp=18`.

Nessuna modifica e stata applicata al plugin C++ o alla semantica del comando
SEA.

## Test e verifiche eseguite

- compilazione Python con `py_compile` dei file coinvolti;
- parsing XML del modello `.osim`;
- parsing JSON della reward imitativa;
- `git diff --check`;
- test isolato del governor con gradino irrealistico:
  - velocita sempre entro `6.0/3.5 rad/s`;
  - accelerazione sempre entro `60/55 rad/s^2`;
- roundtrip della configurazione risolta training/rollout;
- oracle OpenSim completo con `Kp=18` e dinamica ibrida;
- analisi diretta degli STO per tracking di coppia, clamp, `tau_input` e
  velocita motore;
- confronto con oracle precedenti `Kp=10` e `Kp=18` non governato.

Il test storico `validation/_gait_clock_absaction_smoke.py` supera tutti i
controlli funzionali, ma conserva una singola asserzione obsoleta: confronta la
lunghezza dell'osservazione actor-only con l'elenco completo actor+critic.

Un piccolo smoke test RLlib non e stato eseguito fino al training a causa di un
problema preesistente dell'ambiente locale Windows Torch/OpenMP, con DLL OpenMP
duplicate.

## Comando di training

Il nuovo training deve partire da zero, perche il reference governor modifica la
dinamica `action -> riferimento` rispetto ai checkpoint precedenti:

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-rllib python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py" --output-dir "runs\baseline_mlp_imit_v3_governed_win" --reward-mode imitation --reward-json "Trajectory Generator\baseline_MLP\reward_imitation.json"
```

## TODO

- [x] Implementare target imitativo phase-based locale/ciclico e coerente in
      posizione/velocita.
- [x] Eliminare il prefisso iniziale clampato tramite inizializzazione coerente.
- [x] Esporre e penalizzare saturazione interna, torque error e stress motore.
- [x] Aggiungere loss command-rate tra step, `qdot_ref`, `qddot_ref` e
      variazione di `u`.
- [x] Eseguire rollout oracle con target corretto.
- [x] Valutare conservativamente `Kp=10` e mantenere il nominale `Kp=18`.
- [ ] Aggiungere una metrica/reward minima di supporto e carico protesico.
- [ ] Correggere l'asserzione obsoleta actor-only/full-observation nel test
      storico.
- [ ] Risolvere il problema locale Torch/OpenMP e completare uno smoke test
      RLlib.
- [ ] Eseguire da zero il training imitativo `TRAIN-IMIT-V3-GOV-WIN`.
- [ ] Validare il rollout della policy appresa contro i gate oracle: clamp SEA,
      tracking imitativo, contatto protesico e reserve.
