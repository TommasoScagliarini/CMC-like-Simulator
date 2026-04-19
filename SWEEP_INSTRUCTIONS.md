# SEA Motor-Driver Parameter Sweep

## Perche esiste questo sweep

L'inner loop PD del plugin SEA deve comportarsi come un **motor driver
trasparente**: dato un riferimento di coppia `tau_ref = u * F_opt`, il motore
deve produrre `tau_spring ≈ tau_ref` nel modo piu pulito e rapido possibile.

Il setup precedente (K=250/Kp=18.5/Kd=10 per il knee) generava chattering
nel `tau_input` perche i termini proporzionale e derivativo del PD interno
avevano magnitudine quasi uguale, cancellandosi parzialmente e lasciando un
residuo ad alta frequenza.

Lo sweep cerca la combinazione ottimale di **K, Kp, Kd** (per knee e ankle
separatamente) parametrizzata tramite `omega_n` (pulsazione naturale) e
`zeta` (rapporto di smorzamento) del sistema linearizzato:

```
Kp = Jm * omega_n^2 / K - 1
Kd = 2 * zeta * Jm * omega_n - Bm
```

A partire dal 2026-04-18 il plugin espone due property XML aggiuntive:

- `max_motor_torque` (default `500` Nm) — bound di saturazione su
  `tau_input`. Prima era hardcoded a 500 Nm; ora e una property (lo sweep
  usa 5000 Nm) per permettere guadagni adeguati senza clamp.
- `derivative_filter_tau` (default `0.0` s) — costante di tempo del
  filtro LPF del primo ordine applicato a `omega_m` e usato nel termine D
  dell'inner loop non-impedance. A `tau_d=0` la logica e bit-per-bit
  identica alla versione precedente (backward-compat).

Lo sweep ora gira anche su `derivative_filter_tau`, in modo da trovare
una combinazione (K, Kp, Kd, tau_d) che soddisfi simultaneamente le
soglie di tracking e di noise fraction (il best candidate del sweep del
2026-04-17 non riusciva a scendere sotto la soglia di noise 0.20 al
ginocchio; la LPF introdotta nel termine D serve a risolvere questo
trade-off).

## File coinvolti

| File | Ruolo |
|------|-------|
| `validation/sea_driver_sweep.py` | Script di sweep |
| `models/Adjusted_SEASEA - Copia_tuned.osim` | Template .osim |
| `data/3DGaitModel2392_Kinematics_q.sto` | Cinematica di riferimento |
| `config.py` | Configurazione simulatore (letta automaticamente) |
| `plugins/SEA_Plugin_BlackBox_mCMC_impedence.dylib` | Plugin C++ SEA |
| `validation/sweep_results/` | Directory output (creata automaticamente) |

## Come lanciare lo sweep

### Prerequisiti

- Python 3.10+ con `numpy` installato
- Il plugin `.dylib` (macOS) o `.dll` (Windows) deve essere compilato con
  le property `max_motor_torque` e `derivative_filter_tau` (richieste dal
  2026-04-18)
- Il modello template e i file di riferimento devono esistere nei path di default

### 1. Dry run (verifica griglia candidati)

```bash
python validation/sea_driver_sweep.py --dry-run
```

Stampa la griglia di candidati con i parametri derivati, senza eseguire
simulazioni. Utile per verificare quanti candidati passano il pre-filtro.

### 2. Quick smoke test (2-3 candidati, finestra breve)

```bash
python validation/sea_driver_sweep.py --quick-smoke --workers 4
```

Esegue 2-3 candidati su una finestra temporale brevissima per verificare
che il codice funzioni end-to-end senza errori.

### 3. Sweep completo (screening + full validation)

```bash
python validation/sea_driver_sweep.py --workers 12
```

Esegue lo sweep in due fasi:

1. **Screening**: tutti i candidati su finestra breve [4.26, 6.55] s
2. **Full validation**: i migliori N candidati su finestra completa [4.26, 11.06] s

### Opzioni CLI principali

| Flag | Default | Descrizione |
|------|---------|-------------|
| `--workers N` | 12 | Numero di processi paralleli |
| `--dry-run` | off | Solo stampa griglia, nessuna simulazione |
| `--quick-smoke` | off | Test rapido su 2-3 candidati |
| `--template PATH` | `models/Adjusted_SEASEA - Copia_tuned.osim` | Modello template |
| `--reference PATH` | `data/3DGaitModel2392_Kinematics_q.sto` | Cinematica riferimento |
| `--max-motor-torque VAL` | 5000 | Clamp motore nei candidati .osim (scritto nel tag XML `max_motor_torque`) |
| `--top-n-full N` | 5 | Quanti candidati promuovere a full run |
| `--python PATH` | auto | Python con opensim (auto-detect envCMC-like) |
| `--screen-t-start` | 4.26 | Inizio finestra screening |
| `--screen-t-end` | 6.55 | Fine finestra screening |
| `--full-t-start` | 4.26 | Inizio finestra full |
| `--full-t-end` | 11.06 | Fine finestra full |

## Output e logging

Lo script logga nel terminale lo stato di avanzamento:

```
[Screening] [12/287] (4.2%) elapsed=45s ETA=1029s K001_run_id score=0.234 ok=True
```

Al termine di ogni fase stampa un report con classifica e parametri.

I risultati vengono salvati in `validation/sweep_results/`:
- `screening_results.csv` - metriche di tutti i candidati
- `full_results.csv` - metriche dei candidati promossi
- `report.md` - report finale con classifica e raccomandazione
- Sottodirectory per ogni candidato con output `.sto` e log

## Griglia corrente

| Parametro | Valori |
|-----------|--------|
| `K_knee`  | 250, 500, 750, 1000 |
| `K_ankle` | 500, 750, 1000, 1500 |
| `omega_n` | 300, 500, 700, 900, 1100, 1400 rad/s |
| `zeta`    | 0.7, 0.85, 1.0, 1.5 |
| `derivative_filter_tau` | 0.0, 0.001, 0.002, 0.005 s |

Totale cartesiano: 1536. Dopo il pre-filtro sul clamp passano ~1056
candidati. Con 12 worker su Windows la fase di screening richiede
~60-90 min sulla finestra 4.26-6.55 s.

## Criteri di accettazione

| Metrica | Soglia |
|---------|--------|
| Simulazione completata | Si |
| max\|tau_input\| | < 4500 Nm |
| Noise fraction tau_input | < 20% |
| Tracking RMS | < 5 deg per giunto |
| max\|u\| | < 0.95 |
| Output finiti | Tutti |

## Scoring

Il punteggio combina 5 metriche (piu basso = migliore):

| Peso | Metrica |
|------|---------|
| 0.40 | Peggior tracking RMS tra i due giunti |
| 0.20 | Media tracking RMS |
| 0.20 | Noise fraction tau_input |
| 0.10 | max\|tau_input\| normalizzato |
| 0.10 | max\|motor_speed_dot\| normalizzato |

## Dopo lo sweep

1. Identificare i parametri vincenti dal report
2. Aggiornare il file `.osim` con i nuovi K, Kp, Kd
3. Aggiornare `config.py` → `sea_stiffness` con i nuovi K
4. Eseguire un full run con `main.py` per validazione finale
