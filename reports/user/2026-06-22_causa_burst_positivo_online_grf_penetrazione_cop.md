# Causa del Burst Positivo Ankle in `asym100`: Penetrazione, COP e Momento Online GRF

Data: 2026-06-22

## Problema

Nei rollout con GRF ibrida, `sym60` produce una coppia `tau_spring` ankle negativa e biomeccanicamente coerente, mentre `asym100` mostra burst positivi importanti pur mantenendo un tracking `served-reference` apparentemente buono.

La domanda da chiarire era se il burst positivo fosse causato da compensazioni nascoste, da un errore di segno della `online_grf`, oppure dalla penetrazione del piede nel terreno.

## Verdettto breve

Il burst positivo di `asym100` e' spiegato dal momento generalizzato prodotto dalla `online_grf` sulla caviglia protesica.

La catena causale piu' coerente con i dati e':

```text
penetrazione alta
-> cambia la contact patch
-> il COP online passa dall'altro lato della caviglia
-> il momento GRF equivalente sulla ankle cambia segno
-> il SEA deve generare coppia positiva per chiudere il bilancio locale
```

Quindi si': la penetrazione e' un trigger/amplificatore molto probabile. Pero' la causa meccanica diretta non e' il valore di penetrazione in se', ma il ribaltamento del braccio COP-caviglia.

## Strategia

Sono state eseguite quattro verifiche:

1. Lettura dell'implementazione `online_grf` e del plugin `OnlineGRFSphereHalfSpaceForce`.
2. Confronto tra `sym60` e `asym100` sulle colonne salvate in `rollout_episode_online_grf.sto`.
3. Bilancio dinamico locale alla caviglia tramite inverse dynamics su stati reali dei rollout.
4. Proiezione geometrica del momento GRF rispetto al centro della caviglia protesica e all'asse del `PinJoint`.

La formula usata per il bilancio dinamico e':

```text
tau_GRF_online = tau_no_left - tau_SEA
```

dove `tau_no_left` e' la coppia richiesta alla caviglia sugli stessi stati, ma senza GRF sinistra applicata. Questo evita ambiguita' manuali su segni, COP e assi locali.

## Evidenze principali

### Burst 1

| rollout | finestra | penetrazione media | `COP_z - ankle_z` | `tau_GRF_online` | `tau_spring` |
|---|---:|---:|---:|---:|---:|
| `sym60` | 15.875-16.116 s | 10.79 mm | -0.0399 m | +7.17 Nm | -6.64 Nm |
| `asym100` | 15.875-16.116 s | 13.45 mm | +0.0266 m | -17.39 Nm | +17.89 Nm |

Nel caso buono (`sym60`) il COP resta dal lato che produce un momento GRF positivo sulla coordinata ankle, quindi il SEA deve produrre coppia negativa.

Nel caso problematico (`asym100`) il COP attraversa la caviglia e finisce dall'altro lato, quindi il momento GRF diventa negativo e il SEA deve diventare positivo.

### Picco `asym100`

Al picco del burst positivo:

```text
t = 15.974 s
tau_SEA            = +21.83 Nm
tau_no_left        = +0.52 Nm
tau_GRF_online     = -21.31 Nm
left Fy online     = 559.0 N
penetrazione       = 13.73 mm
COP_z - ankle_z    = +31 mm
```

Il punto chiave e' che `left Fy` e' positiva, ma il braccio rispetto alla caviglia ha segno opposto rispetto a `sym60`. Quindi il segno della forza verticale non basta: conta il momento proiettato sull'asse della caviglia.

### Stesso istante in `sym60`

```text
t = 15.974 s
tau_SEA            = -8.56 Nm
tau_no_left        = +0.43 Nm
tau_GRF_online     = +9.00 Nm
left Fy online     = 229.7 N
penetrazione       = 11.41 mm
COP_z - ankle_z    = -45 mm
```

Qui il braccio COP-caviglia mantiene il segno atteso, quindi la coppia SEA resta negativa.

## Sfere di contatto dominanti

Nel picco `asym100`, la sfera dominante e':

```text
online_grf_left_basis_03
normal force = 244.1 N
penetrazione = 13.73 mm
COP = (+0.3329, -0.0095, -0.7738)
```

In `sym60`, allo stesso istante, dominano invece:

```text
online_grf_left_basis_05
normal force = 73.1 N
penetrazione = 11.41 mm

online_grf_left_basis_01
normal force = 58.3 N
penetrazione = 9.31 mm
```

`left_basis_03` e' una sfera piu' arretrata e piu' grande nel frame del piede. Il suo caricamento dominante in `asym100` e' compatibile con un contatto online diverso, piu' profondo e con COP spostato.

## Ruolo del residual/free moment

Il profilo online GRF usato e':

```text
online_grf_profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json
```

Il plugin calcola:

```text
normalForce ~ penetration^1.5
force = normalForce * normal + friction + normalForce * residualForceRatio
freeMoment = normalForce * residual_moment_ratio_m
momentAboutGround = contactPoint x force + freeMoment
```

Il termine residual/free moment amplifica il momento sbagliato, perche' scala con la forza normale. Tuttavia la scomposizione mostra che il solo braccio COP-GRF ribalta gia' il segno in `asym100`; il residuale peggiora la magnitudine, ma non e' necessario per spiegare il cambio di segno.

## Interpretazione

Il tracking ankle resta buono perche' la policy/controller non sta violando la cinematica locale: sta compensando dinamicamente una GRF online che, in quelle finestre, applica alla caviglia un momento di segno opposto a quello atteso.

Quindi il burst positivo non e' una "coppia inutile" generata casualmente dal SEA. Nel modello ibrido con online GRF applicata a sinistra, in quei campioni la coppia positiva e' richiesta dal bilancio dinamico locale.

Questo spiega anche perche' non basta guardare `Fy`: la forza verticale puo' essere positiva in entrambi i casi, ma se il COP passa dall'altro lato della caviglia il momento generalizzato cambia segno.

## Test e verifiche eseguite

- Letto `online_grf.py` per confermare che le colonne salvate includono forza, momento, COP, normal force, penetrazione e contatto.
- Letto `tools/online_grf_contact/OnlineGRFSphereHalfSpaceForce.cpp` per verificare formula di penetrazione, forza normale, residual force ratio e free moment.
- Analizzati i rollout:
  - `Trajectory Generator/runs/rollout/baseline_mlp_imit_v4_c2_4hz_obs_target_resume_reward_norm/sim_outputs`
  - `Trajectory Generator/runs/rollout/MLP_imitation_rollout_06-17-2026_asym_actor_critic_100/sim_outputs`
- Eseguita inverse dynamics su stati reali dei rollout con tre modelli:
  - online sinistra applicata;
  - sinistra senza GRF;
  - prescribed come riferimento diagnostico.
- Verificato il momento geometrico rispetto al centro ankle protesico:
  - centro ankle da `/bodyset/tibia_pylon` con offset `(0, -0.43, 0)`;
  - asse ankle da `PinJoint pros_ankle`;
  - momento proiettato sull'asse del giunto.
- Non sono stati creati output persistenti fuori dai report. Eventuali path `/private/tmp` usati come `output_dir` non contenevano artefatti finali da conservare.

## File modificati

Solo questo report:

```text
reports/user/2026-06-22_causa_burst_positivo_online_grf_penetrazione_cop.md
```

Nessuna modifica a codice, plugin, configurazioni di training o profili `online_grf`.

## Implicazioni

La priorita' successiva non e' modificare la reward sulla coppia ankle. Prima bisogna validare o correggere il modello online GRF:

- controllare se la penetrazione oltre 10 mm e' accettabile o fuori range di calibrazione;
- capire perche' `left_basis_03` prende il carico dominante in `asym100`;
- confrontare COP online vs COP prescribed/oracle nella stessa fase;
- valutare un guardrail runtime su penetrazione/contact patch prima di usare `online_grf` come segnale reward forte.

Fino a questa verifica, `online_grf` non dovrebbe essere usata come segnale forte nella reward biomeccanica.

