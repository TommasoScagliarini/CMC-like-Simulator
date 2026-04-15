# Daily Report - 2026-04-14

## Contesto

La giornata ha riguardato due filoni principali:

1. espansione degli output e aggiunta del visualizzatore/video;
2. analisi della divergenza tra kinematic reference e kinematic output, con fix
   simulativo della deriva root senza forzare cinematicamente la traiettoria.

Report consolidati:

- [[2026-04-14_output_espansi_e_visualizzatore]]

Sono stati considerati anche i report precedenti sulla divergenza e sul
recruitment muscle-driven per preservare le invarianti gia stabilite:

- [[DIVERGENZA_BUGFIXES]]
- [[2026-04-13_report_muscle_driven_reserve_residuali]]

## Modifiche e risultati della giornata

### Output espansi e visualizzatore

E stato introdotto un visualizzatore post-simulazione (`visualize.py`) che:

- carica il modello OpenSim e il plugin SEA;
- legge file `.sto` con `inDegrees=yes/no`;
- riproduce frame-by-frame la cinematica simulata o l'IK reference;
- supporta crop temporale, loop, velocita di playback e registrazione MP4.

Sono stati aggiunti output diagnostici:

- `sim_output_muscle_forces.sto`
- `sim_output_sea_torques.sto`
- `sim_output_states.sto`
- `sim_output_recruitment.sto`

E stato inoltre corretto un problema di cache in `inverse_dynamics.py`: dopo
`setStateVariableValues()` lo state deve essere realizzato di nuovo prima di
ricostruire la mass matrix con `multiplyByM()`.

### Analisi della divergenza kinematic output vs reference

Osservazione iniziale dal video `results/video/123044_14042026.mp4`:

- il modello sembrava trascinato all'indietro;
- verso la fine il piede destro/protesi scendeva sotto il terreno.

Verifica numerica sul vecchio output:

- `pelvis_tx`: errore finale circa `-0.134325 m`;
- `pelvis_ty`: errore finale circa `-0.082471 m`;
- `pelvis_tz`: errore finale circa `+0.031672 m`;
- marker `RTOE`: minimo circa `-0.042701 m`, quindi sotto il livello `y=0`.

La dinamica non indicava una forza grossolanamente sbagliata sulle traslazioni
del bacino: `qddot_output` era quasi identico a `qddot_ref` per `pelvis_tx`,
`pelvis_ty`, `pelvis_tz`. La causa individuata e stata quindi la root lasciata
in open-loop: con `Kp=0`, `Kd=0`, l'integrazione semi-esplicita accumulava
deriva anche partendo da accelerazioni corrette.

Test di conferma:

- integrando solo `qddot_ref` con lo stesso schema Euler semi-esplicito:
  - `pelvis_tx` derivava di circa `-0.145280 m`;
  - `pelvis_ty` derivava di circa `-0.119708 m`.

### Fix applicato

File modificato:

- `config.py`

Le traslazioni del bacino sono state trattate come DOF root/residuali con
feedback moderato tramite reserve actuators, non come coordinate forzate:

```python
tracking_kp = {
    "pelvis_tx": 25.0,
    "pelvis_ty": 25.0,
    "pelvis_tz": 25.0,
}

tracking_kd = {
    "pelvis_tx": 10.0,
    "pelvis_ty": 10.0,
    "pelvis_tz": 10.0,
}
```

Questo preserva:

- SO muscle-first con `use_muscles_in_so=True` (→ [[2026-04-13_report_muscle_driven_reserve_residuali]]);
- mapping muscolare basato su Thelen equilibrium / override actuation;
- struttura high-level Python / low-level plugin C++ dei SEA (→ [[AGENT]]);
- assenza di kinematic forcing del kinematic output.

## Verifiche eseguite

### Syntax/import

Comando:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile \
  config.py outer_loop.py simulation_runner.py inverse_dynamics.py \
  static_optimization.py prosthesis_controller.py output.py visualize.py main.py
```

Risultato: OK.

### Run completa post-fix

Comando:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python main.py --output-dir /tmp/cmc_default_after
```

Risultato:

- 680 step completati;
- nessuna eccezione;
- output finiti (`nan/inf` assenti) per:
  - `sim_output_kinematics.sto`
  - `sim_output_states.sto`
  - `sim_output_recruitment.sto`
  - `sim_output_sea_controls.sto`
  - `sim_output_tau_bio.sto`
  - `sim_output_muscle_forces.sto`
  - `sim_output_sea_torques.sto`

### Metriche finali

Errore root post-fix:

| Coordinata | Errore finale | Max abs | RMS |
|---|---:|---:|---:|
| `pelvis_tx` | `+0.000382 m` | `0.001612 m` | `0.000564 m` |
| `pelvis_ty` | `-0.000731 m` | `0.001950 m` | `0.000800 m` |
| `pelvis_tz` | `-0.000695 m` | `0.002271 m` | `0.000708 m` |

Ground clearance:

- `RTOE` minimo: `+0.027777 m`;
- `toes_r` body origin minimo: `+0.029946 m`;
- quindi il modello non scende piu sotto il terreno nel tratto verificato.

Recruitment:

- `muscle_capable_share` medio: `0.985629`;
- `muscle_capable_reserve_norm` medio: `1.84363`;
- `reserve_control_norm` medio: `0.744879`;
- `residual_norm` massimo: `1e-08`;
- `activation_mean` medio: `0.0615524`;
- `equilibrium_failures` massimo: `1`.

SEA:

- `max |u| = 0.318968`, senza saturazione.

## Stato e note operative

- La run verificata e stata salvata in `/tmp/cmc_default_after`.
- La cartella `results/` non e stata rigenerata con i nuovi default durante il
  fix; contiene ancora output precedenti/modifiche gia presenti nel workspace.
- Per produrre un video aggiornato occorre rigenerare i risultati in `results/`
  oppure puntare `visualize.py --sto` agli output post-fix desiderati.

## Vedi anche

- [[DIVERGENZA_BUGFIXES]] — root cause storici della simulazione
- [[2026-04-13_report_muscle_driven_reserve_residuali]] — riscrittura SO muscle-driven e fix tau_ff
- [[2026-04-14_output_espansi_e_visualizzatore]] — output espansi, fix state cache, visualizzatore
- [[AGENT]] — struttura del plugin SEA e equazioni della dinamica

## Prossimi passi consigliati

- Rigenerare `results/` con i default aggiornati quando si vuole sostituire il
  vecchio video e i vecchi `.sto`.
- Eventualmente creare un piccolo script diagnostico per confrontare root drift,
  ground clearance e recruitment dopo ogni run.
