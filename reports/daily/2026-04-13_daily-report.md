# Daily Report - 2026-04-13

Instruction check token: `CMC_AGENT_OK_2026`

## Sorgente consolidata

- `reports/user/2026-04-13_report_muscle_driven_reserve_residuali.md`

## Sintesi

La giornata ha riguardato la trasformazione della static optimization biologica
in una pipeline realmente muscle-first, con reserve actuators usate come
supporto residuale, e la diagnosi della divergenza osservata intorno a
`t ~= 4.60 s`.

Il contributo muscolare e stato ricostruito frame-by-frame a partire dal
comportamento Thelen-equilibrium e applicato ai muscoli OpenSim tramite
`overrideActuation`. I DOF root `pelvis_*` sono stati distinti dai DOF coperti
dai muscoli, sia nei bound delle reserve sia nelle metriche diagnostiche.

La causa pratica del collasso non era soltanto nel recruitment: il runner
calcolava e passava il feed-forward protesico `tau_ff`, ma la legge effettiva
del controller SEA non lo utilizzava. Il termine e stato ripristinato senza
modificare l'interfaccia high-level Python / low-level plugin C++ ne la
semantica del comando normalizzato inviato al plugin.

## Problema

I problemi affrontati sono stati:

1. Il tracking biologico era stabile, ma le reserve potevano diventare il
   canale principale per produrre `tau_bio`, quindi il risultato non era
   realmente muscle-driven.
2. Il precedente mapping basato soltanto su `moment_arm * Fmax` non descriveva
   adeguatamente i muscoli Thelen, la cui forza dipende anche dallo stato della
   fibra e dall'equilibrio muscolo-tendine.
3. Reimpostare direttamente l'equilibrio delle fibre nello stato forward a ogni
   frame poteva spingere alcuni muscoli verso limiti di lunghezza e rendere non
   finiti gli input della static optimization.
4. La divergenza intorno a `4.60 s` compariva anche in una prova reserve-only:
   il problema non poteva quindi essere attribuito soltanto alla nuova SO
   muscolare.
5. Il controller SEA ignorava nella legge effettiva il `tau_ff` gia calcolato e
   passato dal runner.
6. Una quota reserve globale elevata poteva essere fuorviante, perche i DOF
   root/residuali `pelvis_*` non sono coperti dai muscoli come i giunti
   biologici.

## Soluzione e strategia

La static optimization e stata resa muscle-first:

- `use_muscles_in_so = True`;
- mappa `A_muscle` stimata frame-by-frame mediante forza
  Thelen-equilibrium;
- contributo muscolare calcolato rispetto a `muscle_min_activation`;
- forza ottimizzata applicata ai muscoli OpenSim con
  `overrideActuation`, evitando il reset instabile delle fibre a ogni frame.

Le reserve sono state rese residuali e differenziate per ruolo:

- `reserve_weight = 1.0e6`;
- `reserve_u_max = 50.0` per joint/reserve biologici;
- `unactuated_reserve_u_max = 1000.0` soltanto per le coordinate residuali
  `pelvis_*`;
- esclusione di `pelvis_*` dal denominatore di `muscle_capable_share`.

Nel controller SEA e stata ripristinata la legge gia prevista dalla firma e
dalla documentazione:

```text
tau_cmd = tau_ff + Kp * e + Kd * edot
u = clip(tau_cmd / F_opt, -1, 1)
```

La diagnosi ha seguito questi passaggi:

1. confronto tra SO muscle-first e comportamento reserve-only per verificare
   che la divergenza non dipendesse esclusivamente dai muscoli;
2. isolamento del feed-forward protesico lungo il percorso inverse dynamics,
   runner e controller;
3. separazione delle responsabilita biologiche, protesiche e numeriche;
4. validazione progressiva con run brevi/intermedie e infine con una run
   completa.

E stata inoltre aggiunta la diagnostica di recruitment in
`sim_output_recruitment.sto`, comprendente coppie target, muscolari e reserve,
quote muscolari, attivazioni, residuo ed eventuali failure di equilibrio.

## File modificati

### `config.py`

- configurazione muscle-first;
- parametri di mapping, attivazione e applicazione della forza muscolare;
- pesi e bound distinti delle reserve;
- configurazione della diagnostica recruitment.

### `model_loader.py`

- cache degli indici degli state variable di attivazione e lunghezza fibra;
- controlli di consistenza sugli state variable muscolari mancanti.

### `inverse_dynamics.py`

- baseline zero-actuator con attivazioni portate a
  `muscle_min_activation` e successivamente ripristinate;
- conservazione dell'azzeramento della deflessione SEA;
- conservazione del bypass senza `realizeAcceleration()`.

### `static_optimization.py`

- mapping muscolare basato su equilibrio Thelen;
- baseline muscolare a `a_min`;
- applicazione della forza tramite `overrideActuation`;
- bound differenziati per reserve articolari e residuali;
- diagnostica completa del recruitment e fallback numerici.

### `simulation_runner.py`

- preparazione della baseline muscolare prima dell'inverse dynamics;
- applicazione delle attivazioni e delle forze muscolari prima del calcolo di
  `udot`;
- buffer, stampa periodica e salvataggio della diagnostica recruitment.

### `prosthesis_controller.py`

- ripristino del termine `tau_ff` nella legge di controllo;
- interfaccia e comando normalizzato verso il plugin lasciati invariati.

## Test e verifiche

Sono state eseguite:

- una run intermedia fino a `t = 4.80 s`;
- una run con diagnostica aggiornata fino a `t = 4.85 s`;
- una run completa con il comando:

```bash
/opt/anaconda3/envs/envCMC-like/bin/python main.py --output-dir /tmp/cmc_muscle_full2
```

Risultato della run completa:

- 680 step completati;
- intervallo simulato `4.26 s -> 11.06 s`;
- nessuna eccezione dovuta ad accelerazioni non finite;
- output controllati tutti finiti:
  - `sim_output_recruitment.sto`;
  - `sim_output_activations.sto`;
  - `sim_output_kinematics.sto`;
  - `sim_output_sea_controls.sto`;
  - `sim_output_tau_bio.sto`.

Metriche principali:

| Metrica | Valore |
|---|---:|
| `muscle_capable_share` media | `0.958` |
| `muscle_capable_share` mediana | `0.99999` |
| `activation_nonzero_fraction` media | `0.446` |
| `activation_mean` media | `0.07497` |
| `residual_norm` massimo | `4e-08` |
| `tau_muscle_norm` media | `72.33` |
| `muscle_capable_reserve_norm` media | `8.34` |
| `unactuated_reserve_norm` media | `83.10` |

L'elevato `unactuated_reserve_norm` e attribuito soprattutto ai DOF
root/residuali, distinti dalle coordinate muscle-capable nella nuova
diagnostica.

## Stato finale

- La pipeline biologica e muscle-first e usa le reserve come supporto
  residuale.
- La divergenza osservata intorno a `4.60 s` e stata superata ripristinando il
  feed-forward protesico nel controller SEA.
- La struttura high-level Python / low-level plugin C++ dei SEA e stata
  preservata.
- Il plugin continua a ricevere il comando normalizzato `u` saturato in
  `[-1, 1]`.
- La run completa termina senza eccezioni e con output numericamente finiti.

## TODO

Il report sorgente non contiene TODO espliciti.

### Limiti noti non vincolanti

Questi punti descrivono limiti tecnici osservati, ma non sono TODO formali:

- in 42 frame su 680 le reserve aiutano anche su DOF classificati
  muscle-capable;
- la pipeline e piu vicina a un comportamento CMC-like muscle-first, ma non e
  ancora un CMC completo con integrazione dinamica esplicita di tutte le fibre
  muscolari;
- i warning relativi alle geometrie del visualizer sono separati dalla
  dinamica del simulatore.
