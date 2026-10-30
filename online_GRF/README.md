# Online GRF: runtime, profili e strumenti

Questa cartella raccoglie il runtime Python, i profili JSON di contatto, il
detector geometrico binario, gli strumenti di calibrazione e verifica e i
sorgenti del plugin C++ per le GRF online. GRF indica le forze di reazione del terreno; COP indica il centro
di pressione. HS e TO indicano rispettivamente heel-strike e toe-off.

Gli strumenti di generazione, calibrazione e verifica sono comandi da lanciare
esplicitamente. Il simulatore carica il profilo selezionato dalla configurazione
e usa [`runtime.py`](runtime.py) per configurare e leggere il contatto online.
Quando il detector binario è abilitato, usa anche il sensore geometrico in
`binary_detector/`. Quest'area appartiene al simulatore CMC-like; le FSM e la
policy rimangono in `Trajectory Generator/`. Il core Python del simulatore è
in [`CMC-like Simulator/`](<../CMC-like Simulator/README.md>); questa cartella
e le altre risorse condivise rimangono nella radice del workspace.

## Struttura

```text
online_GRF/
  runtime.py
  profiles/
    AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json
    AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json
    AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json
  binary_detector/
    binary_phase_detector.py
    operational/heel_toe_v25.json
    __init__.py
    README.md
  calibration/       # generazione e calibrazione dei profili di forza
  contact/           # suola da mesh e record congelati del progetto primary GRF
  detector/          # geometria e diagnostica del detector basato sulle forze
  validation/        # confronto dei risultati, verifica plugin e acceptance
  sources/
    CMakeLists.txt
    OnlineGRFSphereHalfSpaceForce.cpp
    OnlineGRFSphereHalfSpaceForce.h
    Plugin_interface.cpp
    README.md
  __init__.py
  README.md
```

I sorgenti del plugin C++ e i comandi CMake sono in
[`sources/`](sources/README.md); le librerie compilate restano
in [`plugins/online_grf_contact/`](../plugins/online_grf_contact/).
Le mesh condivise sono in [`models/Geometry/`](../models/Geometry/).
La cartella [`validation/` del repository](../validation/README.md) contiene
invece i test e il validatore generale del simulatore.

## Profili di forza e geometria binaria

Nei nomi abbreviati della tabella è omesso `AB06_SEASEA_stiff321_500_pi_`.
I link puntano ai file completi.

| File | Contenuto e ruolo |
| --- | --- |
| [online_full_wrench_residual_tangent_v2.json](profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json) | Profilo di forza con 20 sfere e correzioni residue di forze/momenti; selezionato dai preset AB06 recenti e dalla configurazione salvata di best47. |
| [grf_correct_magnitude.json](profiles/AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json) | Byte-identico al precedente, ma con un nome ancora usato da default, validatori e configurazioni salvate. Conservare entrambi i riferimenti. |
| [grf_detector_HS-TO.json](profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json) | Profilo a quattro sfere usato dalle configurazioni che scelgono il detector basato sulle forze. |
| [binary_detector/operational/heel_toe_v25.json](binary_detector/operational/heel_toe_v25.json) | Piano del terreno e due punti solidali al piede. Restituisce contatto binario dalla distanza dal piano; l'interpretazione degli eventi è affidata alla FSM. |

I profili a sfere definiscono geometria e legge di contatto. Il JSON binario
definisce due punti e un piano: non calcola forze. Leggere il
[README del detector binario](binary_detector/README.md) per schema, identità e
compatibilità con i percorsi salvati.
Il modulo runtime
[`binary_detector/binary_phase_detector.py`](binary_detector/binary_phase_detector.py)
legge lo stato OpenSim e restituisce i contatti tallone/punta. L'interpretazione
degli eventi e le FSM rimangono in
[`Trajectory Generator/phase_tracking/`](<../Trajectory Generator/phase_tracking/README.md>).

Il comportamento fisico dipende anche dalla configurazione del simulatore:

| Configurazione | Uso delle forze |
| --- | --- |
| `grf_mode: prescribed` | Il sostegno proviene dalle forze prescritte negli `ExternalLoads`. |
| `grf_mode: online_sensor`, senza lati applicati | Le GRF online sono misurate senza applicare le relative forze al modello. |
| `grf_mode: online_sensor`, con `online_grf_applied_side: [left]` nel preset | Il contatto online viene applicato a sinistra e le forze prescritte di quel lato sono disabilitate; l'altro lato conserva il sostegno prescritto. Nel `SimulatorConfig` Python il campo è `online_grf_applied_sides`. |
| `grf_mode: online` | Il contatto online applica le forze al modello. |

Per esempio, il
[preset ex-novo AB06](<../Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml>)
seleziona il profilo di forza tangent_v2, la geometria binaria V25 e il sostegno
ibrido sinistro. Il profilo effettivo di un rollout va letto nel suo preset,
negli argomenti CLI o in `training_cfg.resolved.yaml`.

## Generazione e calibrazione — `calibration/`

| Script | Funzione |
| --- | --- |
| [generate_online_grf_profile.py](calibration/generate_online_grf_profile.py) | Genera un profilo iniziale dai marker di tallone e punta del modello. |
| [calibrate_online_grf_basis.py](calibration/calibrate_online_grf_basis.py) | Seleziona una combinazione ridotta di contatti da una griglia di candidati, adattandola a forze e momenti misurati. Accetta anche stati di simulazione salvati. |
| [calibrate_online_grf_intensive.py](calibration/calibrate_online_grf_intensive.py) | Calibra in due fasi: forza verticale/tempi di contatto, poi attrito e velocità della superficie. |
| [calibrate_online_grf_residual.py](calibration/calibrate_online_grf_residual.py) | Stima correzioni limitate di forze e momenti usando stati salvati. Il residuo del contatto è distinto dalle reserve del simulatore. |
| [tune_online_grf_support.py](calibration/tune_online_grf_support.py) | Scala la rigidezza per lato usando il rapporto tra impulso online e impulso prescritto riportato da una verifica dinamica. |

Questi comandi possono produrre nuovi profili. Calibrazione e valutazione su
dati separati sono riportate dagli strumenti che le supportano; un buon
adattamento alle forze misurate va accompagnato dalla verifica dinamica del
contatto applicato.

## Suola da mesh — `contact/`

| File | Funzione |
| --- | --- |
| [build_primary_grf_mesh_sole.py](contact/build_primary_grf_mesh_sole.py) | Costruisce un candidato di suola a più sfere, vincolato alla mesh e al protocollo primary GRF: sostegno online sinistro e prescritto destro. |
| [primary_grf_candidate_grid_v1.json](contact/primary_grf_candidate_grid_v1.json) | Griglia congelata di geometrie/materiali, regole di selezione e identità degli input. È letta dal builder. |
| [primary_grf_model_compatibility_audit_2026-07-24.json](contact/primary_grf_model_compatibility_audit_2026-07-24.json) | Report del confronto geometrico tra il modello usato per IK e il modello SEA dinamico dell'esperimento. |
| [primary_grf_surface_velocity_audit_2026-07-24.json](contact/primary_grf_surface_velocity_audit_2026-07-24.json) | Report che documenta la direzione +Z scelta per la velocità del nastro nei dati AB06. |

Il builder verifica gli hash degli input e consente la pubblicazione dei
candidati soltanto in `online_GRF/contact/experimental/`, creata
quando si genera un candidato. Rifiuta la sovrascrittura di un file esistente.
I due audit JSON sono risultati già calcolati, non controlli eseguiti a ogni
avvio. Tutti e tre i record congelati mantengono i percorsi storici interni e i
byte originali; il builder legge la griglia dal percorso corrente qui sopra.

## Detector basato sulle forze — `detector/`

| Script | Funzione |
| --- | --- |
| [audit_two_sensor_prescribed_geometry.py](detector/audit_two_sensor_prescribed_geometry.py) | Analizza sfere heel/toe, mesh del piede e COP; fornisce anche helper geometrici usati da builder e test. |
| [build_two_sensor_mesh_profile_v4.py](detector/build_two_sensor_mesh_profile_v4.py) | Genera un candidato sperimentale V4 conservando il tallone e ricavando la posizione della punta dalla mesh. |
| [validate_online_grf_events.py](detector/validate_online_grf_events.py) | Valuta tempi e forma degli eventi HS/TO ricavati dalle forze online. |
| [validate_two_sensor_prescribed_replay.py](detector/validate_two_sensor_prescribed_replay.py) | Ricostruisce i segnali heel/toe sulla IK e li invia alla FSM two-sensor, confrontandoli con eventi estratti dalle GRF prescritte. |
| [validate_two_sensor_forward_states.py](detector/validate_two_sensor_forward_states.py) | Esegue un controllo analogo su stati di simulazione già salvati. |

Questa famiglia riguarda il precedente detector a sfere/forze. Il percorso
binario V25 è documentato separatamente in `binary_detector/`.
Alcuni input predefiniti di questi strumenti appartengono a campagne ritirate:
per ripeterle occorrono gli artefatti originali, oppure input espliciti
compatibili con il comando scelto.

## Verifica del contatto — `validation/`

| File | Funzione |
| --- | --- |
| [validate_online_grf.py](validation/validate_online_grf.py) | Calibra o valuta un profilo tramite replay IK, confrontandolo con le GRF prescritte e separando calibrazione e holdout. |
| [verify_online_grf_plugin.py](validation/verify_online_grf_plugin.py) | Confronta forze e momenti del plugin C++ con il calcolo Python sugli stessi stati. |
| [compare_online_grf_output.py](validation/compare_online_grf_output.py) | Confronta un output GRF salvato con gli `ExternalLoads` prescritti. |
| [check_online_grf_cop_rollover.py](validation/check_online_grf_cop_rollover.py) | Controlla il movimento del COP dal tallone alla punta, la sua regolarità e le penetrazioni su stati salvati. |
| [validate_online_grf_forward_drift.py](validation/validate_online_grf_forward_drift.py) | Analizza risultati dinamici salvati: deriva rispetto alla IK, sostegno, penetrazione e confronto tra condizioni online/sensor/prescribed. |
| [online_grf_acceptance.py](validation/online_grf_acceptance.py) | Riunisce report di plugin, sensore e dinamica in un esito PASS/FAIL. |
| [online_grf_acceptance_thresholds.json](validation/online_grf_acceptance_thresholds.json) | Soglie del comando acceptance; non modifica i parametri fisici del contatto. |

Le soglie di acceptance e i limiti runtime del preset sono configurazioni
distinte. Un esito riguarda il profilo, gli input e le condizioni valutate nel
report, non tutti i modelli o tutte le velocità di cammino.

## Comandi

Eseguire dalla radice del repository con un ambiente Python dotato di OpenSim,
NumPy e SciPy, per esempio `envCMC-like` su questo Mac. Alcune diagnostiche
richiedono anche Matplotlib. Le righe seguenti sono valide sia nelle shell
macOS sia in PowerShell; sostituire i percorsi di esempio con i propri input.

Esaminare le opzioni di un comando:

```sh
python online_GRF/calibration/calibrate_online_grf_basis.py --help
python -m online_GRF.validation.verify_online_grf_plugin --help
```

Generare un profilo iniziale e calibrarlo contro le GRF prescritte:

```sh
python online_GRF/calibration/generate_online_grf_profile.py --model "path/to/model.osim" --out "online_GRF/profiles/my_model.json"
python online_GRF/validation/validate_online_grf.py --setup "path/to/setup.xml" --profile "online_GRF/profiles/my_model.json" --out-profile "online_GRF/profiles/my_model_calibrated.json" --report "results/my_model_grf_validation.json"
```

Per il secondo comando, `--no-calibrate` valuta un profilo senza eseguire il
fitting. La calibrazione basis può usare `--states-sto` per lavorare su stati
salvati; la calibrazione residual lo richiede.

Verificare il profilo AB06 conservato contro il plugin installato:

```sh
python online_GRF/validation/verify_online_grf_plugin.py --setup "models/AB06/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml" --profile "online_GRF/profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json" --report "results/online_grf_plugin_audit.json"
```

Il comando scrive un report; i profili candidati e i report hanno le
destinazioni esplicitate negli argomenti CLI. Gli esempi che leggono un modello
SEA richiedono anche la libreria SEA compatibile con l'installazione OpenSim.
Per compilazione e verifica del plugin GRF, leggere [`sources/README.md`](sources/README.md).
Per il plugin SEA, leggere [`tools/README.md`](../tools/README.md).

## Percorsi e storia

Dal 2026-10-20 questa area riunisce `online_grf.py`, `online_grf_profiles/` e
`tools/online_GRF/`. Il runtime è `online_GRF.runtime`; gli strumenti si
importano come `online_GRF.<cartella>.<modulo>`. I tre profili di forza sono
in `profiles/`; la geometria binaria V25 resta sotto `binary_detector/operational/`.
I sorgenti e i JSON conservano la stessa logica e gli stessi parametri fisici;
spostare i sorgenti non ricompila né sostituisce le librerie installate.

Il detector si importa come `online_GRF.binary_detector.binary_phase_detector`.
Per caricare manualmente pickle che citano `online_grf`, importare prima
`online_GRF.runtime`; per quelli che citano `binary_phase_detector` o il suo
precedente nome qualificato, importare prima il detector canonico. Questi moduli
registrano i relativi alias in memoria. Il package non carica automaticamente
OpenSim o il detector e non ricrea i vecchi file sul disco.

[`path_resolver.py`](<../CMC-like Simulator/path_resolver.py>) riconosce i
vecchi indirizzi `online_grf_profiles/` dei
profili e la posizione precedente della geometria V25 salvata nei checkpoint,
anche con separatori Windows. I manifest, i
report e le configurazioni congelate conservano i percorsi e gli hash delle
esecuzioni originali. Per riprodurre una campagna ritirata serve il suo checkout
con gli stessi input; non riscrivere quei record per certificare il layout nuovo.

La pulizia ha rimosso i vecchi candidati di forza e i profili opzionali V4 e
sensor-basis. Il V4 può essere rigenerato con il relativo builder, mentre
sensor-basis richiede una calibrazione dagli input originali. La rigenerazione
produce nuovi metadati: per recuperare gli esatti byte storici usare il backup
o la corrispondente versione Git.

Riferimenti:

- [Verifiche della riorganizzazione del 2026-10-17](../reports/cleanup/2026-10-17_grf_tools_reorganization/verification.json).
- [Rimozione dei profili di validazione](../reports/user/2026-10-15_rimozione_finale_profili_grf_validazione.md).
- [Prima pulizia dei profili](../reports/user/2026-10-15_pulizia_profili_grf_online.md).
