# Verifica indipendente dei modelli dopo la riorganizzazione

Esito: PASS per conservazione e rilocazione. Dei 7.060 file originari, tutti sono presenti: 7.040 sono byte-identici e 20 XML corrispondono esattamente agli SHA-256 attesi per le sole modifiche dei percorsi. Tutti i 22 OSIM e i 6.977 asset numerici (MAT, MOT, STO, TRC, CSV) sono byte-identici. Il solo file aggiunto rispetto all’inventario dei modelli è `models/README.md`.

I 116 riferimenti XML sono stati risolti nuovamente: tutti conservano la stessa destinazione rimappata, senza nuovi collegamenti rotti. Le cinque famiglie risultano AB06, AB07, AB08, SEASEA e SEASEA_healthy.

## Riferimenti input già mancanti

I 21 riferimenti input già mancanti prima della migrazione restano gli stessi. Sono 9 riferimenti nei tre setup healthy; 4 `ExtLoads.xml` assenti nei template raw (AB06, AB07, AB08 e AB06_SEASEA); 3 percorsi Windows Dropbox nei setup IK originali raw; 2 riferimenti AB06 statici al precedente nome inesistente `models/AB06_SEASEA`; 3 file IK assenti richiesti come input dai setup ID convertiti. I dataset alternativi non sono stati sostituiti automaticamente.

- `models/AB06/AB06-raw/osimxml/idsetup.xml` · `external_loads_file` → `ExtLoads.xml`
- `models/AB06/AB06-raw/osimxml/iksetup.xml` · `model_file` → `D:\Dropbox (GaTech)\PAPERS\AmbulationBiomechanics\PAPER\SubmissionJBiom\Review\CODE\DATA\SPLIT\AB06\osimxml\AB06.osim`
- `models/AB06/AB06_SEASEA-raw/data/converted/static/static_01/static_01_iksetup.xml` · `model_file` → `../../../../../../AB06_SEASEA/osimxml/AB06_SEASEA_marker_calibrated.osim`
- `models/AB06/AB06_SEASEA-raw/data/converted/static/static_01/static_01_iksetup.xml` · `marker_file` → `../../../../../../AB06_SEASEA/data/converted/static/static_01/static_01.trc`
- `models/AB06/AB06_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_idsetup.xml` · `coordinates_file` → `treadmill_01_01_ik.mot`
- `models/AB06/AB06_SEASEA-raw/osimxml/idsetup.xml` · `external_loads_file` → `ExtLoads.xml`
- `models/AB07/AB07-raw/osimxml/idsetup.xml` · `external_loads_file` → `ExtLoads.xml`
- `models/AB07/AB07-raw/osimxml/iksetup.xml` · `model_file` → `D:\Dropbox (GaTech)\PAPERS\AmbulationBiomechanics\PAPER\SubmissionJBiom\Review\CODE\DATA\SPLIT\AB07\osimxml\AB07.osim`
- `models/AB07/AB07_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_idsetup.xml` · `coordinates_file` → `treadmill_01_01_ik.mot`
- `models/AB08/AB08-raw/osimxml/idsetup.xml` · `external_loads_file` → `ExtLoads.xml`
- `models/AB08/AB08-raw/osimxml/iksetup.xml` · `model_file` → `D:\Dropbox (GaTech)\PAPERS\AmbulationBiomechanics\PAPER\SubmissionJBiom\Review\CODE\DATA\SPLIT\AB08\osimxml\AB08.osim`
- `models/AB08/AB08_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_idsetup.xml` · `coordinates_file` → `treadmill_01_01_ik.mot`
- `models/SEASEA_healthy/Adjusted_newmarkers_fp12_15889_19839_setup.xml` · `kinematics_file` → `models/SEASEA_healthy/data/IK_fp12_15889_19839.mot`
- `models/SEASEA_healthy/Adjusted_newmarkers_fp12_15889_19839_setup.xml` · `external_loads_xml` → `models/SEASEA_healthy/data/ExternalForces_fp12_15889_19839.xml`
- `models/SEASEA_healthy/Adjusted_newmarkers_fp12_15889_19839_setup.xml` · `reserve_actuators_xml` → `models/SEASEA_healthy/data/CMC_Actuators_fp12_15889_19839.xml`
- `models/SEASEA_healthy/Adjusted_newmarkers_setup.xml` · `kinematics_file` → `models/SEASEA_healthy/data/3DGaitModel2392-scaled_Kinematics_q.sto`
- `models/SEASEA_healthy/Adjusted_newmarkers_setup.xml` · `external_loads_xml` → `models/SEASEA_healthy/data/ExternalForces.xml`
- `models/SEASEA_healthy/Adjusted_newmarkers_setup.xml` · `reserve_actuators_xml` → `models/SEASEA_healthy/data/CMC_Actuators.xml`
- `models/SEASEA_healthy/gait2176_L_TFP_ORIGINALE_newmarkers_setup.xml` · `kinematics_file` → `models/SEASEA_healthy/data/3DGaitModel2392-scaled_Kinematics_q.sto`
- `models/SEASEA_healthy/gait2176_L_TFP_ORIGINALE_newmarkers_setup.xml` · `external_loads_xml` → `models/SEASEA_healthy/data/ExternalForces.xml`
- `models/SEASEA_healthy/gait2176_L_TFP_ORIGINALE_newmarkers_setup.xml` · `reserve_actuators_xml` → `C:\Users\tomma\Desktop\Opensim OMNIBUS\3D_Model_Leg_and_Prosthesis_Completo_21_76\CMC\CMC_Actuators.xml`

## Destinazioni output non ancora presenti

Queste sono distinte dagli input mancanti: 10 destinazioni output non presenti, già nella baseline (9 file e una directory). Comprendono 6 output IK, 3 output ID e il vecchio results_directory AB06 statico. Un output non ancora generato non prova la perdita di un input.

- `models/AB06/AB06_SEASEA-raw/data/converted/static/static_01/static_01_iksetup.xml` · `results_directory` → `../../../../../../AB06_SEASEA/data/converted/static/static_01`
- `models/AB06/AB06_SEASEA-raw/data/converted/static/static_01/static_01_iksetup.xml` · `output_motion_file` → `static_01_ik.mot`
- `models/AB06/AB06_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_idsetup.xml` · `output_gen_force_file` → `treadmill_01_01_id.sto`
- `models/AB06/AB06_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_iksetup.xml` · `output_motion_file` → `treadmill_01_01_ik.mot`
- `models/AB07/AB07_SEASEA-raw/data/converted/static/static01/static01_iksetup.xml` · `output_motion_file` → `static01_ik.mot`
- `models/AB07/AB07_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_idsetup.xml` · `output_gen_force_file` → `treadmill_01_01_id.sto`
- `models/AB07/AB07_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_iksetup.xml` · `output_motion_file` → `treadmill_01_01_ik.mot`
- `models/AB08/AB08_SEASEA-raw/data/converted/static/static01/static01_iksetup.xml` · `output_motion_file` → `static01_ik.mot`
- `models/AB08/AB08_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_idsetup.xml` · `output_gen_force_file` → `treadmill_01_01_id.sto`
- `models/AB08/AB08_SEASEA-raw/data/converted/treadmill/treadmill_01_01/treadmill_01_01_iksetup.xml` · `output_motion_file` → `treadmill_01_01_ik.mot`

## Qualifica del modello healthy

I tre OSIM healthy si caricano e inizializzano sia prima sia dopo la riorganizzazione. I tre setup storici non sono pronti a un rollout completo: la cartella data era già vuota e i nove riferimenti a cinematica, GRF e reserve erano già mancanti. La rinomina aggiorna i percorsi di questa famiglia senza ricostruire o inventare i dati scientifici.

Il confronto nativo dei tre soggetti attivi AB06/AB07/AB08 copre 30 passi / 30 ms ciascuno: 48 STO, 52.920 valori confrontati, tutti identici numericamente e byte per byte. Non costituisce validazione scientifica di un episodio completo né prova nativa Windows.
