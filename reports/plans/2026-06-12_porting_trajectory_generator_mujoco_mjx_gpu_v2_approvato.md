# Porting Trajectory Generator su MuJoCo/MJX GPU — piano rivisto (v2, approvato)

> Revisione approvata della bozza `2026-06-12_porting_trajectory_generator_mujoco_mjx_gpu.md`.
> Modifiche principali rispetto alla bozza: correzione della premessa sul modello (mismatch 321/500_pi vs 1000/700 verificato sul codice), aggiunta della Fase 0 di fattibilità GPU come gate-zero bloccante, riscoping del v1 a "engine + parità + benchmark senza training", riuso di una libreria PPO JAX esistente, separazione dei gate di parità per livello, e break-even di benchmark esplicito.

## Contesto

L'obiettivo è usare `MuJoCo_env` come **backend di produzione parallelizzato su GPU** (RTX 4070 via WSL2/JAX/MJX) per accelerare training e inference della Trajectory Generator, mantenendo OpenSim come **oracolo scientifico**. Si porta lo stato attualmente implementato di `baseline_MLP` (RLlib PPO + ambiente CMC-like). `Prosthesis_SNN`, i TODO aperti e la conversione dei checkpoint RLlib restano fuori scope.

Il porting deve replicare le **scelte strutturali** della Trajectory Generator (idea, non strumento: es. RLlib → libreria PPO JAX va bene), non la dinamica fisica, che invece deve restare 1:1 con OpenSim.

**Correzione di premessa, verificata sul codice.** La parità dinamica NON è raggiunta per l'obiettivo RL, perché i due lati usano modelli diversi:

- Il `baseline_MLP` si addestra su `AB06_SEASEA_stiff321_500_pi_setup.xml` → modello `AB06_SEASEA_stiff321_500_pi.osim`, stiffness SEA **321/500**, controllo **PI** cascade con anti-windup (SOURCE: `train_ppo_mlp.py:43`, `training_cfg.yaml:37`, `prosthesis_controller.py:355-414`).
- `MuJoCo_env` è stato convertito da `Adjusted_SEASEA - Copia_tuned.osim`, stiffness **1000/700** (TARGET: `mujoco_env/sea_plugin.py:61`, `CONTEXT.md`). Le metriche di parità che oggi "passano" (q_pros RMSE ≈ 0.0087 rad) sono contro il **modello sbagliato** e non si trasferiscono all'obiettivo.

Quindi la parità va **ricostruita da zero** contro il setup 321/500_pi. Inoltre l'`Ki·xi` del "pi" vive nel loop esterno cascade sulla velocità (`candidate_cmd = p_cmd + ki·xi`), non sommato nel `tau_input` del plugin: il percorso esatto del segnale va tracciato prima di portarlo.

**Decisioni prese (questa revisione):**
1. **v1 = engine MJX batched + gate di parità + benchmark, SENZA training.** Si prova lo speedup prima di costruire PPO.
2. **PPO (fase post-v1) riusa una libreria JAX esistente** (PureJaxRL/rejax/sbx/Brax), non implementazione Flax/Optax da zero.
3. **La strategia per la static optimization nel hot-path si decide dopo lo spike** (Fase 0), sui numeri reali di throughput/fedeltà.

---

## Fase 0 — Spike di fattibilità GPU (GATE-ZERO, bloccante)

Prima di scrivere qualsiasi cosa di prodotto, de-rischiare i single-point-of-failure. Se uno fallisce, il piano va ridisegnato.

- **Ambiente**: JAX-CUDA + `mujoco-mjx` gira sulla RTX 4070 in WSL2/Ubuntu (`requirements-gpu-linux.txt`). Verificare `jax.devices()` → GPU e uno `mjx.step` vmappato.
- **Tendon Jacobian on-device**: verificare se `mjx.Data`/`mjx` espone l'equivalente di `ten_J` per gli spatial tendon. È un **prerequisito hard**: senza, la static optimization del lato biologico non gira in GPU. Fallback da misurare: differenze finite via `mjx` o moment arm precomputati.
- **Attuatori muscolari MJX**: confermare che `mjDYN_MUSCLE` + `mjTRN_TENDON` siano supportati/equivalenti in MJX o vadano sostituiti.
- **Static optimization vmappata**: micro-benchmark di `jaxopt.OSQP` sotto `jit`+`vmap` a batch 1/32/256, confrontando **iterazioni fisse (no backtracking)** vs semantica backtracking. Output: scelta della strategia (decisione #3).
- **Float32 e stiffness**: verificare la stabilità della molla SEA rigida (Ks ≤ 1000) a 1 ms in float32; stabilire se serve substepping più fine o `x64` in modalità validation.

**Deliverable Fase 0**: nota tecnica con esiti GO/NO-GO per ciascun punto e la strategia QP scelta.

---

## Fase 1 — Sincronizzazione modello e dinamica (321/500_pi)

- **Ri-conversione MJCF** dal setup corrente con `tools/osim2mjcf.py`, partendo da `AB06_SEASEA_stiff321_500_pi.osim` (non più `Adjusted_SEASEA`). Aggiornare `kinematic_couplings.json` e i metadati stiffness.
- **Import sorgenti RL correnti**: IK `.mot`, `ExternalForces.xml`, `CMC_Actuators.xml`, profilo online-GRF JSON, parametri controller — quelli referenziati da `training_cfg.yaml` SOURCE.
- **Manifest con hash** di tutte le sorgenti OpenSim usate (modello, IK, GRF, attuatori, profilo GRF), come artefatto versionato.
- **Dinamica SEA reale "pi"**: portare stato integrale, anti-windup, `Ki`, limite integrale, `Kd_Imp`, clamp e diagnostica, fedeli a `prosthesis_controller.py` SOURCE (`_cascade_integral_step`, `_clamp_cascade_integral_torque`, `_apply_u_lpf`). Tracciare il percorso esatto del segnale integrale.
- **GRF ibrida equivalente**: contatto MJX sul lato protesico + GRF prescritta sul lato sano (mappa `online_grf.py:385-414` SOURCE → `physics_mode="contact"` parziale in TARGET). Sotto-fase con gate proprio.
- File TARGET chiave: `mujoco_env/sea_plugin.py`, `mujoco_env/prosthesis_controller.py`, `mujoco_env/config.py`, `mujoco_env/model_loader.py`, `tools/osim2mjcf.py`.

---

## Fase 2 — Ambiente CMC-like JAX/MJX batched (cuore v1)

- **API pubbliche pure-JAX**: `CMCEnvConfig`, `BatchedEnvState`, `reset_batch()`, `step_batch()`.
- **Stato persistente nel pytree batched**: `mjx.Data`, SEA, controller, static optimization, reference governor, gait clock, azione precedente, RNG (chiavi JAX), stato episodio.
- **Esecuzione**: policy-step da 10 ms via `jax.lax.scan` sui substep fisici da 1 ms, `jax.vmap` sugli environment. Niente NumPy/SciPy/conversioni Python/host-callback nel percorso caldo.
- **Static optimization**: strategia scelta in Fase 0; warm-start; fallimento → **truncation esplicita**, niente fallback SciPy nascosto. Riscrivere il feasibility-backtracking (`simulation_runner.py:819-854` TARGET) come `lax.scan` a iterazioni fisse con selezione `jnp.where`, oppure eliminarlo (per decisione Fase 0).
- **Riscritture JIT necessarie** (già mappate): loop `action_repeat` di `trajectory_env.py` → `lax.scan`; `_advance_rk4_bypass_step` → integrazione MJX pura; densificazione `ten_J` → `scatter`/`scan`; `while` in `sea_step_substeps` → `lax.while_loop`; osservazione con `jnp`.

**Contratto strutturale della Trajectory Generator da replicare 1:1** (idea, non strumento):
- azione assoluta `(policy_knots=3, 2)` con anchor continuo e interpolazione cubica (PCHIP-equivalente) — SOURCE `osim_trj_cmc_like.py:1414-1489`;
- reference model 6 Hz, governor persistente, limiti velocità/accelerazione (knee 6.0/60.0, ankle 3.5/55.0) — SOURCE `:307-554`;
- target imitativo periodico phase-based (shift knee 0.465 / ankle 0.452) — SOURCE `:650-779`;
- osservazione actor-realistic + critic-privileged con ordine stabile — SOURCE `:1621-1780`. **Definire un unico spec di indici condiviso tra env e modulo PPO**, con test di parità campo-per-campo;
- raw reward terms e reward centralizzata `ex_novo`/`imitation` con i pesi di `reward_function.py` SOURCE;
- diagnostica/penalità SEA, command-rate, safety, OOB (banda gait su traiettoria comandata, non simulata);
- gait clock da heel-strike GRF, GRF online, terminazioni e `end_reason` equivalenti (fall / joint_divergence / grf_penetration / time / numerical / timeout).
- File TARGET chiave: `mujoco_env/trajectory_env.py`, `mujoco_env/mjx_runner.py`, `mujoco_env/simulation_runner.py`, `mujoco_env/static_optimization.py`, `mujoco_env/outer_loop.py`.

---

## Fase 3 — Parità & benchmark (deliverable v1)

**Gate di parità separati per livello** (evita asticelle irraggiungibili):

- **A. Equivalenza di formula** (deterministica, quasi-esatta): unit test che, dati input identici, SEA/governor/action-mapping/reward/static-opt producono lo stesso output OpenSim↔MJX. Qui valgono tolleranze strette.
- **B. Parità dinamica closed-loop** (azioni scriptate identiche, OpenSim vs MuJoCo CPU): `q_pros_rmse ≤ 0.01`, `q_bio_rmse ≤ 0.02`, `sea_torque_rmse ≤ 5 Nm`; terminazioni/`end_reason` identici; contatto protesico concorde ≥95%; heel-strike/toe-off MAE ≤ 20 ms; normal GRF RMSE ≤ 0.10 BW; penetrazione < 0.028 m. **I gate su reward/term sono qui closed-loop, con tolleranza larga**, non bit-exact.
- **C. MuJoCo CPU vs MJX**: parità N=1 single policy-step `rtol=1e-4`/`atol=1e-5`; parità comportamentale su episodi; env batched indipendenti, reset mascherati, riproducibilità per seed.

**Benchmark con break-even esplicito** (criterio di successo v1, non solo informativo): definire a priori il target, es. **≥5× env-steps/s vs OpenSim 12-worker a batch 256**. Misurare compile time, post-compile env-step/s e policy-step/s, memoria, su batch 1/8/32/128/256 e massimo compatibile.

La correlazione attivazioni resta **diagnostica e non bloccante**; attivazioni/recruitment non entrano in reward/osservazioni finché il mismatch non è chiuso.

---

## Fase 4 — Training PPO (post-v1, fuori dal primo rilascio)

- **Riuso di libreria PPO JAX esistente** (PureJaxRL/rejax/sbx/Brax) sopra `step_batch()`; l'actor-critic asimmetrico è una patch di slicing sull'osservazione (`obs[:n_actor]` policy, full per il value).
- `training_cfg.yaml` come sorgente unica + `training_cfg.resolved.yaml`; checkpoint best/last via Orbax; `summary.json`, cronologia iterazioni, TensorBoard.
- Supervisione, timeout, restart da ultimo checkpoint valido; STO/diagnostica solo fuori dal loop JIT.
- RLlib resta baseline OpenSim, non backend di produzione; nessuna conversione/ripresa dei checkpoint RLlib.

---

## Verifica end-to-end

1. **Fase 0**: eseguire lo spike su WSL2/4070 e produrre la nota GO/NO-GO. Senza GO su ten_J + OSQP vmappata, fermarsi e ridisegnare.
2. **Fase 1**: `python tools/osim2mjcf.py` sul nuovo setup; `python tools/validate_model.py` deve passare la Fase 1 geometrica sul modello 321/500.
3. **Fase 2-3**: estendere `tests/` con unit test di equivalenza-formula (gruppo A) e i test di parità closed-loop (gruppo B) e MJX (gruppo C); harness di parità riusando `mujoco_env/parity.py` (`compare_outputs`, allineamento colonne per nome).
4. **Benchmark**: script batch (estendere `scripts/run_mjx_batch.py`) che riporta throughput post-compilazione e confronto con OpenSim/CPU; verifica del break-even.
5. Tutto il path GPU su Linux/WSL2; Windows nativo resta CPU/debug.

## Consegna

- v1 consegna l'**engine MJX batched con parità e benchmark verificati**; il training PPO è pianificato come fase successiva.
- OpenSim resta la fonte di verità scientifica; MuJoCo è il backend veloce.
- L'approvazione di questo piano non avvia automaticamente modifiche al codice.

## Riferimenti runtime
- [JAX installation](https://docs.jax.dev/en/latest/installation.html)
- [NVIDIA CUDA on WSL](https://docs.nvidia.com/cuda/wsl-user-guide/index.html)
