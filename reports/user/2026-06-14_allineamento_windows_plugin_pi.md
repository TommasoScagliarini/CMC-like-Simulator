# Allineamento Windows del plugin PI SEA

Data: 2026-06-14

## Problema

Il modello `AB06_SEASEA_stiff321_500_pi.osim` conteneva gia' i parametri del
motor driver PI e della plant SEA, ma su Windows il simulatore caricava una DLL
precedente che esponeva soltanto `Kp`, `Kd`, `motor_angle` e `motor_speed`.
Di conseguenza, le property `Ki` e `integral_torque_limit` presenti nel modello
non erano operative e lo stato `torque_error_integral` non esisteva nel runtime
Windows.

L'obiettivo era allineare Windows alla procedura validata su macOS, installare
il plugin PI e rendere persistente come default la configurazione AB06 PI con
outer loop cascade.

## Soluzione e strategia

La procedura macOS documentata nei report del 16-17 maggio e' stata ricostruita
e applicata a Windows:

1. verificato che il sorgente in `tools/sea_plugin_relative_d/` contiene le
   property `Ki`, `integral_torque_limit`, lo stato OpenSim
   `torque_error_integral` e l'output `torque_error_integral_dot`;
2. compilata e verificata una DLL Windows Release isolata;
3. salvata la DLL Windows precedente come backup;
4. installata la DLL PI come plugin attivo;
5. eseguita una simulazione CMC-like completa sulla finestra `11.99-16.0 s`;
6. resi persistenti il setup AB06 PI, il controller cascade e i fallback della
   plant SEA.

Backup della DLL precedente:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll.bak_pre_pi_20260614_182653
```

Hash SHA-256:

```text
DLL precedente: FA39998F075E6AB6E276C67888323E7F19B032089319A9C4EF7A3A7A9E0BDA43
DLL PI attiva:  092664F947F9A0D0F707A02B7EAF905434C8C85E24D99B9020A57CD68FFB2ABE
```

## Configurazione resa default

### Outer loop cascade Python

| Giunto | Kp_outer | Kp_inner | Ki_inner | I_torque_limit |
|---|---:|---:|---:|---:|
| Knee | 18.85 | 29.2 | 1377.0 | 50.0 Nm |
| Ankle | 47.125 | 2.8275 | 213.0 | 200.0 Nm |

Legge:

```text
qdot_cas = qdot_ref + Kp_outer * (q_ref - q)
tau_cmd  = Kp_inner * (qdot_cas - qdot)
         + Ki_inner * integral(qdot_cas - qdot) dt
```

### Motor driver PI nel plugin C++

| SEA | Kp | Kd | Ki | integral_torque_limit |
|---|---:|---:|---:|---:|
| SEA_Knee | 18.0 | 11.0 | 190.0 | 100 Nm |
| SEA_Ankle | 11.3 | 11.0 | 123.0 | 100 Nm |

Il plugin e' in modo `Impedence=false` e applica il clamp finale a
`+/-500 Nm`.

La legge effettivamente implementata e validata nel plugin corrente e':

```text
tau_input_raw = tau_ref
              + Kp * (tau_ref - tau_spring)
              + clamp(Ki * xi, -integral_torque_limit, +integral_torque_limit)
              - Kd * omega_m
```

Il termine feedforward `+tau_ref` e' presente nel plugin validato. Rimuoverlo
modificherebbe la dinamica del motor driver e richiederebbe ricompilazione e
nuova validazione.

### Plant SEA

| SEA | F_opt | K | Jm | Bm |
|---|---:|---:|---:|---:|
| SEA_Knee | 100 Nm | 321 Nm/rad | 0.01 kg m^2 | 0.10 Nm s/rad |
| SEA_Ankle | 250 Nm | 500 Nm/rad | 0.01 kg m^2 | 0.10 Nm s/rad |

## Sorgenti dei parametri

- `config.py`: controller outer cascade, fallback stiffness, bundle e modello
  AB06 PI predefiniti.
- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim`: parametri
  motor driver PI e plant SEA.
- `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml`:
  modello, cinematica, GRF, reserve e finestra predefinita `11.99-21.0 s`.
- `setup_io.py`: usa il setup AB06 PI come fallback cross-platform quando non
  esiste uno stato locale valido.
- `plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll`: DLL PI Windows attiva.

Il setup XML attualmente ricordato in `.simulator_last_setup.json` coincide con
il setup reso default:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml
```

## File modificati

- `config.py`
  - bundle e modello default spostati su AB06 PI;
  - cinematica e ExternalLoads default allineati al bundle AB06;
  - finestra default impostata a `11.99-21.0 s`;
  - fallback stiffness aggiornato a knee `321`, ankle `500`;
  - documentazione aggiornata da inner PD a inner PI.
- `setup_io.py`
  - aggiunto fallback persistente al setup AB06 PI.
- `main.py`
  - documentato il nuovo comportamento di fallback.
- `plugins/SEA_Plugin_BlackBox_mCMC_impedence_ff.dll`
  - installata la build Windows PI validata.

Il modello PI conteneva gia' i parametri richiesti ed e' stato preservato.

## Test e verifiche

### Caricamento plugin e modello

Il caricamento OpenSim reale della DLL attiva ha confermato:

- `Ki` e `integral_torque_limit` disponibili per entrambi i SEA;
- 6 state variable SEA totali:
  `motor_angle`, `motor_speed`, `torque_error_integral` per knee e ankle;
- tutti i parametri motor driver e plant uguali ai valori richiesti.

### Simulazione CMC-like Windows

Comando:

```powershell
conda run -n envCMC-like python main.py `
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml `
  --t-start 11.99 `
  --t-end 16.0 `
  --output-dir results/_windows_pi_alignment_11.99_16 `
  --validate `
  --log
```

Risultato:

- simulazione completata a `t=16.0 s`;
- `4010` step;
- RK4 ha integrato tutti i 6 stati SEA;
- integrali PI e derivate finiti e variabili;
- nessuna saturazione `tau_input`.

| Verifica PI | Knee | Ankle |
|---|---:|---:|
| Range `torque_error_integral` | 0.32304963 | 0.20539629 |
| Range `torque_error_integral_dot` | 15.6838928 | 27.342637 |
| Max errore plugin/Python `tau_input` | 5.0e-7 Nm | 5.0e-6 Nm |
| Saturazione `tau_input` | nessuna | nessuna |

Il validatore globale ha prodotto un unico FAIL su `mtp_angle_r` per tracking
oltre soglia. Il FAIL e' relativo al tracking biologico globale e non
all'allineamento del plugin PI; tutti i controlli specifici SEA/PI sono PASS.

## Esito

L'allineamento Windows del plugin PI e' completato. La DLL PI resta installata,
il setup AB06 PI e' il default effettivo e i parametri outer loop, motor driver
e plant SEA risultano coerenti tra configurazione, setup, modello e runtime
OpenSim.

