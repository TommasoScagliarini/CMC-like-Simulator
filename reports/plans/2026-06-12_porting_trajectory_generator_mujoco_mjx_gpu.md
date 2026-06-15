# Porting Trajectory Generator MLP su MuJoCo/MJX GPU

## Obiettivo

Usare OpenSim come oracle scientifico e `MuJoCo_env` come backend di produzione
per training e rollout veloci sulla RTX 4070 tramite WSL2/JAX/MJX.

Il primo rilascio portera lo stato **attualmente implementato** di
`baseline_MLP`. `Prosthesis_SNN`, TODO aperti e conversione dei checkpoint
RLlib restano esclusi.

## Stato iniziale

- La parita dinamica non e ancora raggiunta.
- `MuJoCo_env` usa un vecchio modello con stiffness SEA `1000/700` e senza
  stato integrale PI.
- Il modello OpenSim corrente usa stiffness `321/500`, gain aggiornati e:

  ```text
  tau_input = tau_ref + Kp*(tau_ref-tau_spring) + Ki*xi - Kd*omega_m
  ```

- MJX supporta attualmente soltanto step fisici grezzi, non l'intero ambiente
  CMC-like batched.
- Il percorso GPU richiedera WSL2/Ubuntu; Windows nativo restera disponibile
  per CPU/debug.

## Implementazione

### 1. Sincronizzazione modello e dinamica

- Rigenerare il bundle MJCF dal setup OpenSim corrente
  `AB06_SEASEA_stiff321_500_pi_setup.xml`.
- Importare modello, IK, ExternalLoads, reserve, parametri controller e profilo
  GRF correnti.
- Salvare un manifest con hash e parametri delle sorgenti OpenSim utilizzate.
- Portare fedelmente la dinamica SEA reale: stato integrale, anti-windup, `Ki`,
  limite integrale, `Kd_Imp`, clamp e diagnostica.
- Correggere la documentazione OpenSim obsoleta senza modificare la semantica
  del plugin.
- Implementare la GRF ibrida equivalente: contatto MuJoCo applicato sul lato
  protesico sinistro e GRF prescritta sul lato sano destro.

### 2. Ambiente CMC-like JAX/MJX batched

- Introdurre `CMCEnvConfig`, `BatchedEnvState`, `reset_batch()` e
  `step_batch()` come API pubbliche JAX.
- Conservare tutto lo stato persistente nel pytree batched: `mjx.Data`, SEA,
  controller, static optimization, reference governor, gait clock, azione
  precedente, RNG e stato episodio.
- Eseguire ogni policy-step da `10 ms` tramite `jax.lax.scan` sui substep fisici
  da `1 ms` e `jax.vmap` sugli environment.
- Eliminare NumPy, SciPy, conversioni Python, callback host e fallback CPU dal
  percorso caldo.
- Usare static optimization JAX warm-started; un fallimento produce una
  truncation esplicita, senza fallback SciPy nascosto.
- Esporre modalita precisione `validation` e `training`.

Portare lo stesso contratto strutturale della Trajectory Generator:

- azione assoluta `(policy_knots=3, 2)` con anchor continuo e interpolazione
  cubica equivalente;
- reference model a `6 Hz`, governor persistente e limiti
  velocita/accelerazione;
- target imitativo periodico phase-based e inizializzazione coerente;
- osservazione actor-realistic e critic-privileged con ordine stabile;
- raw reward terms e reward centralizzata `ex_novo`/`imitation`;
- diagnostica e penalita SEA, command-rate, sicurezza e OOB;
- gait clock, GRF online, terminazioni e motivi di fine episodio equivalenti.

### 3. PPO JAX e UX

- Implementare PPO JAX-native con Flax/Optax e checkpoint Orbax.
- Supportare attore simmetrico e actor-critic asimmetrico.
- Mantenere `training_cfg.yaml` come sorgente unica e salvare
  `training_cfg.resolved.yaml`.
- Conservare artefatti equivalenti: checkpoint best/last, policy inference,
  `summary.json`, cronologia iterazioni e TensorBoard.
- Conservare supervisione, timeout e restart da ultimo checkpoint valido.
- Fornire comandi semplici per training, oracle, rollout batch e rollout
  deterministico batch `1`.
- Registrare STO e diagnostica soltanto fuori dal loop JIT, selezionando gli
  environment richiesti.
- Non convertire ne riprendere checkpoint RLlib esistenti. RLlib resta
  disponibile come baseline OpenSim, non come backend di produzione MJX.

## Gate di parita

Il training GPU rimane bloccato finche non passano i gate policy-relevant.

OpenSim contro MuJoCo CPU, con azioni scripted identiche:

- `q_pros_rmse <= 0.01 rad`;
- `q_bio_rmse <= 0.02 rad`;
- `sea_torque_rmse <= 5 Nm`;
- reward scalare MAE `<= 0.01`;
- errore relativo dei reward term attivi `<= 5%`, con tolleranza assoluta per
  termini prossimi a zero;
- terminazioni e `end_reason` identici;
- contatto protesico concorde almeno al `95%`;
- heel-strike/toe-off MAE `<= 20 ms`;
- normal GRF RMSE `<= 0.10 BW`;
- penetrazione sempre sotto `0.028 m`.

MuJoCo CPU contro MJX:

- parita N=1 di un policy-step con `rtol=1e-4`, `atol=1e-5`;
- parita comportamentale su episodi completi;
- environment batched indipendenti, reset mascherati e riproducibilita per
  seed.

La correlazione diretta delle attivazioni resta diagnostica e non bloccante,
ma attivazioni/recruitment non potranno entrare in reward o osservazioni finche
il relativo mismatch non sara chiuso.

## Verifiche e benchmark

- Unit test delle formule SEA, governor, action mapping, reward e static
  optimization.
- Oracle imitativo OpenSim/MuJoCo con gli stessi gate biomeccanici.
- Test MJX N=1, multi-step, multi-env e checkpoint round-trip.
- Smoke training PPO simmetrico e asimmetrico.
- Rollout deterministico singolo e rollout batch.
- Verifica JAX CUDA/MJX sulla RTX 4070 in WSL2.
- Benchmark documentato, senza soglia minima bloccante, per batch `1`, `8`,
  `32`, `128`, `256` e massimo compatibile con la memoria.
- Confronto di compile time, post-compile env-step/s, policy-step/s e memoria
  contro OpenSim 12-worker e MuJoCo CPU.

## Assunzioni e consegna

- OpenSim resta la fonte di verita scientifica; MuJoCo e il backend veloce.
- Il primo rilascio include solamente `baseline_MLP` e feature gia
  implementate.
- SNN, TODO aperti e checkpoint RLlib saranno pianificati separatamente.
- L'approvazione di questo piano non avvia automaticamente modifiche al codice.

## Riferimenti runtime

- [JAX installation](https://docs.jax.dev/en/latest/installation.html)
- [NVIDIA CUDA on WSL](https://docs.nvidia.com/cuda/wsl-user-guide/index.html)
