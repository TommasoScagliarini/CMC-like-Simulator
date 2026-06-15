# Piano operativo: implementazione e lancio training imitativo V4

**Data:** 2026-06-14  
**Obiettivo:** rendere disponibile entro oggi una configurazione V4 validata e
lanciabile per il training imitativo.

## Decisioni bloccate

- Mantenere policy update rate a `100 Hz` (`segment_duration=0.01`).
- Mantenere `gamma=0.95`.
- Ridurre l'azione a un comando posizione assoluto per knee e ankle
  (`policy_knots=1`).
- Sostituire il reference model corrente con un modello causale di terzo ordine,
  continuo in posizione, velocita e accelerazione, con jerk limitato.
- Usare inizialmente banda reale `-3 dB = 4 Hz`.
- Rimuovere dall'attore la phase normalizzata dell'episodio e il valore scalare
  discontinuo della gait phase; mantenere seno/coseno della gait phase.
- Separare nella reward imitativa:
  - qualita del riferimento generato `served-target`;
  - imitazione effettiva `actual-target`;
  - tracking di esecuzione `actual-served`.
- Avviare il primo training V4 da zero.

## Piano di implementazione

1. Estendere `CMCEnvConfig` con tipo/banda/jerk limit del reference model.
2. Implementare stato continuo `[q_ref, qdot_ref, qddot_ref]` e integrazione
   jerk-limited a `1 ms`.
3. Fare in modo che con `policy_knots=1` la policy emetta solamente `q_cmd`.
4. Esporre stato del reference model nell'osservazione actor.
5. Rimuovere phase di episodio e gait phase scalare dall'attore.
6. Calcolare e registrare:
   - `served_imitation_loss`;
   - jerk e jerk loss;
   - diagnostica del reference model.
7. Aggiornare `reward_function.py` e la reward imitativa V4.
8. Collegare i nuovi parametri a YAML, CLI, snapshot e rollout.
9. Correggere inizializzazione `motor_speed` SEA coerente con `joint_speed`.
10. Creare configurazione YAML dedicata al training V4.

## Decisione sul reset imitativo

Il confronto oracle ha mostrato che `imitation_initialize_to_target=false`
introduce un errore iniziale inevitabile molto grande mentre il reference model
converge dalla cinematica prescribed al target anti-fase. La V4 mantiene quindi
`imitation_initialize_to_target=true`, ma inizializza ora anche la velocita
motore SEA con la velocita del giunto. Il reset risulta coerente in:

- posizione giunto e `motor_angle`;
- velocita giunto e `motor_speed`;
- posizione e velocita del riferimento imitativo;
- accelerazione iniziale del reference model pari a zero.

## Validazione minima prima del lancio

- `py_compile` sui file modificati.
- `git diff --check`.
- validazione configurazione YAML/snapshot;
- test isolato del reference model:
  - continuita `q`, `qdot`, `qddot`;
  - jerk entro limite;
  - attenuazione della riga a `100 Hz`;
- smoke environment con azione `policy_knots=1`;
- oracle imitativo breve con assenza di clamp SEA;
- smoke RLlib, se l'ambiente locale lo permette.

## Gate per autorizzare il training

- action space coerente con due output policy;
- osservazioni finite e schema stabile;
- nessun errore di configurazione training/rollout;
- reference model continuo in accelerazione;
- jerk limit rispettato;
- nessun clamp SEA nell'oracle breve;
- nessuna regressione evidente del tracking `actual-served`;
- comando di lancio completo e riproducibile.

## Configurazione candidata

```yaml
ppo:
  gamma: 0.95

simulation:
  segment_duration: 0.01
  policy_knots: 1
  episode_duration: 2.0
  random_init: false
  pros_ref_model: butterworth3_jerk_limited
  pros_ref_cutoff_hz: 4.0
```

I jerk limit saranno inizializzati conservativamente e verificati tramite
oracle prima del training.

## Fallback

Se il reference model C2 non supera i gate entro la sessione:

1. non lanciare un training lungo con contratto parzialmente modificato;
2. conservare modifiche e risultati diagnostici;
3. riportare esattamente il gate bloccante;
4. lasciare il comando di smoke/oracle e il prossimo passo operativo.

## Stato

- [x] Piano operativo scritto.
- [x] Contratti env/config/reward mappati.
- [x] Reference model C2 implementato.
- [x] Action/observation V4 implementate.
- [x] Reward/config V4 implementate.
- [x] Validazioni completate.
- [x] Comando training pronto.

## Risultati validazione

- `py_compile`: superato sui file modificati.
- `git diff --check`: superato.
- Config V4 risolta correttamente: `policy_knots=1`, `gamma=0.95`,
  Butterworth terzo ordine a `4 Hz`.
- Test reference model con comandi alternati:
  - salto massimo ai confini: `q=0`, `qdot=2.8e-13`,
    `qddot=2.7e-9`;
  - massimi rispettati: `6 rad/s`, `60 rad/s2`, `3000 rad/s3`.
- Oracle OpenSim breve con `Kp_knee=18`: nessun clamp SEA.
- Smoke RLlib completo: build, sampling, aggiornamento PPO, checkpoint e
  snapshot completati.
- Smoke RLlib parallelo con `2` EnvRunner: completato.
- Smoke inference dal checkpoint: completato e configurazione V4 ricostruita
  automaticamente.
- Audit osservazione actor: nessuna phase scalare; restano solo rappresentazioni
  cicliche seno/coseno per gait clock e fase del sensore protesico.

Il validatore legacy `validation/validate_training_config.py` supera tutti i
controlli prima del test snapshot, che nel sandbox corrente fallisce per i
permessi Windows di `TemporaryDirectory`; il round-trip snapshot e stato
comunque validato dallo smoke training/inference reale.

## Comando di lancio

```powershell
C:\Users\tomma\anaconda3\Scripts\conda.exe run --no-capture-output -n envCMC-rllib python "Trajectory Generator\baseline_MLP\train_ppo_mlp.py" --config "Trajectory Generator\baseline_MLP\training_cfg.v4_imitation.yaml" --output-dir "runs\baseline_mlp_imit_v4_c2_4hz"
```

Non usare `--resume-from`: action space, observation space e reward sono
cambiati rispetto alla V3.
