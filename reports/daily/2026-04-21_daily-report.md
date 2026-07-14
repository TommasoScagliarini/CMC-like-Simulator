# Daily Report - 2026-04-21

Instruction check token: CMC_AGENT_OK_2026

## Contesto

Consolidamento retrospettivo dei lavori del 21 aprile rimasti solo nei report
utente. Il daily consolidato del 20 aprile copriva gia' la diagnosi dei gain
outer e il confronto `20/2` contro `5/0.5`, ma non includeva il filtro
cinematico a 6 Hz e lo sweep outer PD-only.

## Report utente consolidati

- `reports/user/2026-04-21_diagnosi_mancato_tracking.md`
- `reports/user/2026-04-21_confronto_outer_gain_20_2_vs_5_0p5.md`
- `reports/user/2026-04-21_implementazione_filtro_lowpass_6hz.md`
- `reports/user/2026-04-21_outer_pd_only_sweep.md`

I primi due sono richiamati anche da
`reports/daily/2026-04-20_daily-report.md`.

## Problemi affrontati

- I gain outer `100/20` chiedevano coppie oltre il budget fisico dei SEA e
  portavano a saturazione e tracking non fisiologico.
- Il file IK aveva campionamento non perfettamente uniforme e contenuto ad alta
  frequenza che la derivazione spline amplificava in `qdot_ref` e `qddot_ref`.
- Dopo la rimozione di `tau_ff` dal comando outer, il vecchio tuning `20/2`
  non era piu' adeguato al contratto PD-only.

## Soluzione e strategia

La cinematica viene ora convertita in radianti, ricampionata a `1 ms`, filtrata
con Butterworth zero-phase del quarto ordine a `6 Hz` e solo dopo trasformata
in spline. Il filtro e' configurabile e puo' essere disabilitato o ritoccato da
CLI.

Per il controller PD-only e' stato creato
`validation/outer_gain_sweep.py`, con screening a stadi di `4608` candidati,
finestre multiple, hard reject fisici e conservazione integrale dei soli full
run finalisti.

## Risultati principali

Sulla finestra lunga `4.26-11.06 s`, il filtro ha ridotto i picchi massimi di
`qddot` del `69-86%`:

| Coordinata | Grezzo | Filtrato 6 Hz |
|---|---:|---:|
| `pros_knee_angle` | `475.805` | `67.812 rad/s^2` |
| `pros_ankle_angle` | `77.382` | `15.706 rad/s^2` |
| `ankle_angle_r` | `319.109` | `97.961 rad/s^2` |
| `mtp_angle_r` | `27.984` | `6.848 rad/s^2` |

Lo sweep PD-only ha superato dry-run e quick smoke su tre candidati. Il full
sweep Windows non e' stato eseguito in questa giornata.

## File modificati o creati

- `config.py`
- `config.yaml`
- `main.py`
- `kinematics_interpolator.py`
- `validation/outer_gain_sweep.py`

## Test e verifiche

- confronto filtro acceso/spento su finestre breve e lunga;
- `py_compile` dei file coinvolti: PASS;
- dry-run sweep: `4608` candidati correttamente enumerati;
- quick smoke OpenSim macOS: 3 candidati, CSV e cleanup `.sto` PASS.

## TODO e stato successivo

- [x] Mantenere il preprocessing IK a 6 Hz come default riproducibile.
- [x] Documentare l'intera sequenza parse, resample, filtro e spline.
- [x] Il full sweep outer PD-only proposto per Windows e' stato superseduto dai
      successivi lavori su feed-forward, PI inner e cascade; non e' una baseline
      corrente da rilanciare senza una nuova decisione sperimentale.
- [x] La diagnosi dei gain outer e' confluita nel tuning successivo: `20/2`
      risultava il compromesso stabile del contratto allora in uso.

