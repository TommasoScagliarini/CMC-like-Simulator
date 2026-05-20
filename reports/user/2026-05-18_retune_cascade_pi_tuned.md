# Retune cascade con modello PI tuned - 2026-05-18

## Problema

La configurazione cascade scelta in mattinata era stata ottenuta usando il
setup/modello PI originale:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi.osim
```

L'utente ha richiesto di rifare i test usando invece:

```text
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi-tuned_setup.xml
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi-tuned.osim
```

La differenza e' nei gain PI low-level dei due SEA. Il modello `pi-tuned`
ha driver motore piu' morbidi:

```text
SEA_Knee originale:  Kp=18,   Kd=11,   Ki=190
SEA_Knee tuned:      Kp=2.13, Kd=4.38, Ki=136

SEA_Ankle originale: Kp=11.3, Kd=11,   Ki=123
SEA_Ankle tuned:     Kp=1.01, Kd=4.38, Ki=87.8
```

Di conseguenza i gain cascade trovati per il PI originale non erano
direttamente trasferibili: sulle prime prove con `pi-tuned` le config piu'
aggressive saturavano `u` o peggioravano il tracking.

Obiettivo: mantenere un tracking cinematico paragonabile al best candidate
di stamattina e ridurre il chattering su motor velocity/motor acceleration e
`tau_input`.

## Strategia

Sono state eseguite tre iterazioni di screening su finestre brevi e due batch
di full run:

```text
root risultati:
results/_cascade_iter_pi_tuned_20260518_192606

setup:
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi-tuned_setup.xml

modello:
models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi-tuned.osim

baseline di confronto:
results/_cascade_full_ankle5_20260518_105345/full_runs/full_ankle5_kpo18p85_kpi29p2_kii1377_kil50_apo47p125_api2p8275_aii213_ail200
```

La metrica di tracking e' lo score cinematico pesato:

```text
score_kinematic_deg = 0.3 * knee_RMS_deg + 0.7 * ankle_RMS_deg
```

Per la scelta finale e' stato guardato in particolare il rapporto medio
high-pass sui segnali direttamente richiesti:

```text
knee/ankle motor_speed_hpf50_rms
knee/ankle motor_speed_dot_hpf50_rms
knee/ankle tau_input_plugin_hpf50_rms
```

## Risultati

Le configurazioni derivate dal PI originale sono state scartate sul modello
`pi-tuned`: erano troppo aggressive per il driver low-level piu' morbido.

La famiglia funzionante ha:

- loop posizione ankle piu' pronto;
- loop velocity/torque ankle meno rigido;
- knee leggermente piu' morbido rispetto alla configurazione del PI originale.

Full run completati:

```text
7 full run totali
7/7 complete
7/7 finite
7/7 senza saturazioni tau_input
```

Il candidato scelto e':

```text
full_s3_mid_a80_v20_i140
```

Configurazione:

```text
knee:
  Kp_outer = 14.0
  Kp_inner = 15.0
  Ki_inner = 500.0
  I_limit  = 50.0

ankle:
  Kp_outer = 80.0
  Kp_inner = 2.0
  Ki_inner = 140.0
  I_limit  = 200.0
```

Confronto con la baseline mattutina:

| Metrica | baseline mattina | pi-tuned scelto | Delta |
|---|---:|---:|---:|
| score cinematico | 0.9075 deg | 0.9169 deg | +1.0 % |
| knee RMS | 0.1776 deg | 0.4596 deg | peggiore, ma <0.5 deg |
| ankle RMS | 1.2203 deg | 1.1128 deg | migliore |
| motor/tau HPF chatter richiesto | 1.000x | 0.516x | -48 % |
| power score | 1.000x | 0.721x | -28 % |
| max `u` | 0.4818 | 0.4772 | OK |
| saturazioni `tau_input` | 0 | 0 | OK |
| max `tau_input_raw` | 124.8 Nm | 118.1 Nm | OK |

Esiste un'alternativa piu' conservativa per il chattering:

```text
full_s3_kfast_a75_v18_i130
```

Questa riduce ancora di piu' il chatter richiesto (`0.301x`) ma peggiora il
tracking ankle (`1.278 deg`) e lo score cinematico (`0.963 deg`). Per questo
non e' stata scelta come default: il candidato `mid_a80` rispetta meglio
l'obiettivo principale di restare vicino al tracking della baseline mattutina.

## Soluzione

Aggiornato `config.py` con i gain del candidato:

```python
sea_cascade_kp_outer = {
    "pros_knee_angle":  14.0,
    "pros_ankle_angle": 80.0,
}

sea_cascade_kp_inner = {
    "pros_knee_angle":  15.0,
    "pros_ankle_angle": 2.0,
}

sea_cascade_ki_inner = {
    "pros_knee_angle":  500.0,
    "pros_ankle_angle": 140.0,
}

sea_cascade_inner_i_torque_limit = {
    "pros_knee_angle":  50.0,
    "pros_ankle_angle": 200.0,
}
```

E' stato aggiunto un commento esplicito che lega questa configurazione al
setup `AB06_SEASEA_stiff321_500_pi-tuned_setup.xml` e al run
`full_s3_mid_a80_v20_i140`.

## File modificati

```text
config.py
```

File risultati/diagnostica creati:

```text
results/_cascade_iter_pi_tuned_20260518_192606/
results/_cascade_iter_pi_tuned_20260518_192606/full_all_ranking.csv
results/_cascade_iter_pi_tuned_20260518_192606/final_selection_summary.json
plot/05_18_2026_4/
```

## Verifiche eseguite

```text
/opt/anaconda3/envs/envCMC-like/bin/python -m py_compile config.py
```

Risultato: OK.

Plot generato con:

```text
plot/05_18_2026_4
```

`missing_channels.txt` riporta:

```text
No missing channels.
```

Tutti i full run selezionati sul modello `pi-tuned` sono terminati
`complete`, con output finiti e senza saturazioni `tau_input`.

## Questioni aperte

La scelta finale privilegia il vincolo di tracking vicino alla baseline.
Se in una fase successiva si decidesse di massimizzare ulteriormente la
riduzione di chattering accettando un tracking ankle un po' peggiore, il
candidato da considerare e':

```text
full_s3_kfast_a75_v18_i130
```
