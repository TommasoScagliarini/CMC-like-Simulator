# Validazione eventi GRF online, fase protesica e implicazioni reward

Data: 2026-06-26

## Problema

L'obiettivo era capire se la `online_grf` potesse essere usata per stimare in
modo affidabile la fase protesica, senza trasformarla in un target quantitativo
di forza. La domanda di partenza era pratica: i punti task-based della reward
che dipendono da stance/swing, carico e unloading richiedono una conoscenza
robusta di HS/TO sul lato protesico. Prima di usare questi segnali in reward o
nell'actor, bisognava validare che gli eventi fossero identificabili online.

Il vincolo concettuale emerso durante la discussione e':

- il deployment finale non potra' contare su un wrench GRF completo;
- l'actor dovrebbe vedere solo segnali compatibili con sensori protesici
  realistici: HS, TO, contatto/fase, eventualmente tempo da ultimo HS;
- magnitudo GRF, COP, momenti e impulse non devono diventare verita' reward se
  non sono validati come misure biomeccaniche.

## Strategia

La validazione e' stata impostata come verifica di eventi, non come validazione
di forza assoluta.

1. Reference prescribed:
   - file: `models/AB06_SEASEA_Threadmill/data/AB06_SEASEA_GRF_FullSpan.mot`;
   - soglia reference: `20 N`;
   - durata minima contatto: `0.05 s`;
   - ciclo minimo: `0.30 s`;
   - finestra setup: `11.99-21.0 s`;
   - `sample_dt = 0.001 s`.

2. Detector online:
   - confronto eventi online vs eventi prescribed con matching uno-a-uno;
   - gate strict sul lato sinistro/protesico:
     - HS precision = 1;
     - HS recall = 1;
     - TO precision = 1;
     - TO recall = 1;
     - HS max abs error <= 50 ms;
     - TO max abs error <= 80 ms.

3. Iterazione:
   - prima sweep sul profilo storico `grf_correct_magnitude`;
   - poi confronto dei profili `online_grf_profiles/*.json` gia' presenti;
   - infine sweep fine sul profilo che passava meglio per HS/TO;
   - conferma anche su stati forward salvati.

## Implementazione

E' stata aggiunta una pipeline dedicata:

```text
validation/validate_online_grf_events.py
```

La pipeline:

- legge setup, prescribed GRF e profili `online_grf`;
- campiona la GRF online in IK replay oppure da `CoordinateStates` salvati;
- applica smoothing causale opzionale su Fy;
- usa il detector streaming gia' presente nel simulatore;
- salva:
  - `summary.json`;
  - `reference_events.csv`;
  - `best_event_matches.csv`;
  - `sweep_ranking.csv`;
  - `profile_ranking.csv`;
  - `online_grf_event_timing.png`;
  - `report.md`.

Sono stati aggiunti anche test sintetici:

```text
validation/test_online_grf_event_matching.py
```

I test coprono:

- perfect match;
- HS spurio;
- TO mancante;
- shift temporale fuori tolleranza;
- duty factor;
- contact interval metrics.

## Risultati della validazione eventi

### Profilo storico: grf_correct_magnitude

Profilo:

```text
online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json
```

Esito: **FAIL** per uso come detector HS/TO.

Prima sweep:

```text
HS matched: 2 / 6
TO matched: 0 / 5
left contact F1: 0.913
left contact IoU: 0.839
```

Sweep con soglie molto basse:

```text
HS matched: 2 / 6
TO matched: 1 / 5
left contact F1: 0.936
left contact IoU: 0.880
```

Interpretazione: il contatto come intervallo grossolano e' discreto, ma gli
eventi HS/TO non sono matchati uno-a-uno. Il profilo storico non e' adatto come
sorgente primaria della fase protesica.

### Profilo migliore per eventi: grf_detector_HS-TO

Profilo:

```text
online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json
```

Detector raccomandato:

```text
low_threshold = 15 N
confirmation_threshold = 120 N
min_contact_duration = 0.03 s
causal_smoothing = 0.10 s
```

Esito IK replay: **PASS**.

```text
Left HS precision/recall = 1.0 / 1.0
Left HS max abs error = 0.013 s

Left TO precision/recall = 1.0 / 1.0
Left TO max abs error = 0.026 s

Left contact F1 = 0.944
Left contact IoU = 0.894
```

Conferma su stati forward salvati:

```text
states file = results/sim_output_states.sto

Left HS precision/recall = 1.0 / 1.0
Left HS max abs error = 0.013 s

Left TO precision/recall = 1.0 / 1.0
Left TO max abs error = 0.027 s

Left contact F1 = 0.944
Left contact IoU = 0.895
```

Quindi, per rilevare fase/eventi protesici, `grf_detector_HS-TO` passa con
un margine ampio rispetto al gate strict.

## Confronto tra profilo storico e profilo scelto per eventi

I due profili non sono equivalenti e non sono stati costruiti per lo stesso
scopo.

### grf_correct_magnitude

Caratteristiche:

- sorgente: `calibrated_state_only_full_wrench_residual`;
- 20 sfere/basis totali;
- treadmill surface velocity nel ground;
- residual force ratio e residual moment ratio scalati dalla normal force;
- target storico: full wrench, supporto dinamico, uso ibrido;
- metadata: `recommended_mode = online_candidate`,
  `online_mode_status = requires_forward_validation`.

Questo profilo era stato scelto perche', nel setup ibrido storico, caricava
davvero la protesi e riduceva le reserve rispetto a un piede scarico.

Dal report storico del 2026-06-08:

```text
ankle SEA tau mean/max: 1.1 / 2.4 Nm -> 30.6 / 53.9 Nm
knee SEA tau mean/max: 2.3 / 4.3 Nm -> 38.4 / 111.9 Nm
pelvis_ty reserve mean: 512 N -> 64 N
```

Quindi la ragione storica non era il timing HS/TO, ma il fatto che la caviglia
protesica non fosse piu' un DOF libero nella dinamica ibrida.

### grf_detector_HS-TO

Caratteristiche:

- sorgente: `calibrated_against_prescribed_grf`;
- 4 sfere totali: heel/toe per lato;
- nessun residuale;
- metadata: `status = preliminary`;
- nota interna: short calibration smoke, da ricalibrare prima di uso production.

Nonostante sia piu' semplice e preliminare, il suo Fy e' piu' leggibile per
eventi HS/TO. Per questo e' emerso come miglior proxy di un sensore di contatto.

## Interpretazione corretta

Non e' corretto dire che `grf_detector_HS-TO` sostituisce automaticamente
`grf_correct_magnitude` nel training dinamico.

La conclusione corretta e':

```text
fase/event detector -> grf_detector_HS-TO PASS
contatto dinamico storico -> grf_correct_magnitude aveva una ragione forte
reward forte su magnitudo GRF -> nessun profilo e' autorizzato automaticamente
```

In altre parole:

- se serve stimare HS/TO/fase, usare `grf_detector_HS-TO + detector`;
- se serve applicare un contatto dinamico ibrido nel simulatore, lo storico
  `grf_correct_magnitude` non va rimpiazzato senza un test A/B dinamico;
- se serve un reward biomeccanico, non usare la magnitudo GRF come target forte.

## Implicazioni per deployment

Il deployment finale non avra' un sensore capace di misurare perfettamente GRF,
COP, momenti e wrench completo. L'ipotesi realistica e' avere un sensore di
contatto/evento sulla protesi.

Per questo, il profilo `grf_detector_HS-TO` non va interpretato come "GRF
fisica migliore", ma come:

```text
proxy simulativo validato di contact/event sensor
```

L'actor dovrebbe ricevere solo feature compatibili con questa sensoristica:

```text
HS pulse
TO pulse
in_contact / stance-swing
time_since_last_HS
estimated prosthetic phase
stance_fraction_hat
```

Non dovrebbe ricevere il wrench completo `online_grf` come se fosse un sensore
realistico.

## Compatibilita con la reward task-based

I punti 1 e 2 della proposta reward vanno reinterpretati.

Versione originale:

1. premiare carico verticale protesico in stance;
2. penalizzare GRF protesica alta in swing.

Versione coerente con sensoristica HS/TO:

1. **stance contact validity**
   - durante stance protesica deve esserci contatto stabile;
   - non si impone una magnitudo Fy target;
   - eventuale Fy minima resta solo guardrail debole o diagnostica.

2. **swing unloading / swing clearance**
   - durante swing protesico non deve esserci contatto;
   - la penalita' puo' essere binaria/contact-based;
   - non serve conoscere la forza precisa.

Le penalty GRF gia' introdotte vanno classificate cosi':

```text
penetration / slip / contact validity:
  guardrail simulativo a basso peso o terminazione di sicurezza

ankle moment flip:
  diagnostica o penalty debole, non reward centrale

load magnitude / impulse / COP / full wrench:
  diagnostica, non reward forte
```

Questa formulazione e' coerente con un actor che conosce solo HS/TO e fase.

## Decisione operativa

La decisione consigliata e':

```text
training dinamico per ora:
  mantenere grf_correct_magnitude come profilo storico applicato,
  finche' non viene fatto un confronto A/B dinamico.

fase protesica / detector / actor observation:
  usare grf_detector_HS-TO + detector validato.

reward:
  usare contatto/fase, non magnitudo GRF, per i punti 1/2.
```

Questo separa due canali:

1. **physics/contact dynamics channel**
   - serve a far stare in piedi la simulazione;
   - puo' restare basato sul profilo storico.

2. **phase/contact sensor channel**
   - serve a stimare HS/TO/fase;
   - deve essere compatibile con sensoristica reale;
   - usa il detector validato.

## File modificati o creati

Creati/modificati durante questa attivita':

```text
validation/validate_online_grf_events.py
validation/test_online_grf_event_matching.py
reports/user/2026-06-26_validazione_eventi_grf_fase_protesica_reward.md
```

Artifact principali prodotti:

```text
results/online_grf_event_validation_grf_correct_magnitude/
results/online_grf_event_validation_grf_correct_magnitude_low_threshold/
results/online_grf_event_validation_profile_compare/
results/online_grf_event_validation_preliminary_fine/
results/online_grf_event_validation_forward_states_recommended/
```

Nessuna modifica intenzionale a:

```text
Trajectory Generator/baseline_MLP/training_cfg.yaml
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
Trajectory Generator/baseline_MLP/reward_function.py
```

## Test e verifiche eseguite

Verifiche software:

```text
/opt/anaconda3/envs/envCMC-rllib/bin/python -m py_compile \
  validation/validate_online_grf_events.py \
  validation/test_online_grf_event_matching.py

/opt/anaconda3/envs/envCMC-rllib/bin/python \
  validation/test_online_grf_event_matching.py

/opt/anaconda3/envs/envCMC-rllib/bin/python -m unittest \
  validation.test_online_grf_event_matching \
  validation.test_online_grf_core
```

Esiti:

```text
test_online_grf_event_matching: 7/7 PASS
test_online_grf_event_matching + test_online_grf_core: 14/14 PASS
git diff --check sui nuovi file: PASS
```

Verifiche numeriche:

```text
IK replay grf_correct_magnitude:
  FAIL

IK replay profile compare:
  PASS con grf_detector_HS-TO

IK replay fine sweep grf_detector_HS-TO:
  PASS, HS max 13 ms, TO max 26 ms

forward states results/sim_output_states.sto:
  PASS, HS max 13 ms, TO max 27 ms
```

## TODO

- [ ] Eseguire confronto A/B dinamico ibrido tra `grf_correct_magnitude` e
  `grf_detector_HS-TO`, misurando ankle/knee torque, reserve, penetration,
  COP, moment flip, contact interval e stabilita'.
- [ ] Decidere se implementare due canali separati nel training:
  profilo storico per contatto applicato e profilo/event detector per fase
  osservata dall'actor.
- [ ] Aggiornare la reward task-based per riformulare i punti 1/2 come
  contact/fase validity invece che carico GRF quantitativo.
- [ ] Evitare l'uso della magnitudo `online_grf` come reward forte finche' non
  esiste una validazione dinamica gait-scale senza compensazioni spurie.
- [ ] Se si promuove il detector, salvare in config i parametri raccomandati:
  `low_threshold=15 N`, `confirmation_threshold=120 N`,
  `min_contact_duration=0.03 s`, `smoothing=0.10 s`.
