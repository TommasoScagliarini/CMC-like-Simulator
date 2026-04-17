# Safe Path Plugin E Diagnosi DLL Exception

Data: 2026-04-16

## Problema

La sessione e ripartita dal report:

```text
reports/user/2026-04-16_stop_validazione_sea_plugin.md
```

Il problema iniziale era validare il simulatore SEA plugin-driven senza
produrre risultati tautologici. In particolare, il vecchio comportamento
forzava:

```text
motor_angle = q + tau_ref / K
motor_speed = qdot
```

e quindi faceva coincidere `tau_spring` e `tau_ref` quasi a precisione macchina.

Durante il lavoro sono comparsi due errori nativi Windows distinti:

1. Access violation:

```text
L'istruzione a ... ha fatto riferimento alla memoria a 0x0000000000000000.
La memoria non poteva essere read.
```

2. Eccezione software sconosciuta:

```text
0xc06d007f
```

Il vincolo operativo dichiarato dall'utente e stato: non accedere a memoria
pericolosa e non usare scorciatoie che alterano la logica fisica del simulatore.

## Interpretazione Degli Errori

### Access Violation

L'access violation era associata al path nativo OpenSim che passa da:

```python
model.realizeAcceleration(state)
model.computeStateVariableDerivatives(state)
model.getStateVariableDerivativeValue(...)
```

Questo path puo entrare in codice C++ nativo non catturabile da Python e, nel
modello corrente, e gia stato osservato come instabile.

La soluzione adottata e stata evitare completamente queste chiamate operative
nel codice Python del simulatore.

### Eccezione `0xc06d007f`

L'errore `0xc06d007f` e diverso dall'access violation. Su Windows e compatibile
con una eccezione nativa da caricamento DLL/procedura non trovata, spesso legata
a mismatch tra:

- plugin C++ compilato contro una versione di OpenSim;
- DLL OpenSim/Simbody effettivamente caricate a runtime;
- runtime MSVC disponibile nel `PATH`.

Controlli statici eseguiti:

- la DLL corrente in `plugins/` e:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence.dll
```

- la build del plugin punta a:

```text
C:/OpenSim-mCMC
```

da:

```text
..\SEA_plugin_core\build\CMakeCache.txt
```

- l'ambiente `envCMC-like` importa OpenSim da:

```text
C:\OpenSim-mCMC\sdk\Python\opensim
```

- il `PATH` dell'ambiente inizia con:

```text
C:\OpenSim-mCMC\bin
```

- sul PC sono presenti anche:

```text
C:\OpenSim 4.1
C:\OpenSim 4.5
```

Quindi l'ipotesi principale per `0xc06d007f` e un caricamento runtime non
coerente se il simulatore viene lanciato fuori dall'ambiente corretto o se
un'altra installazione OpenSim entra prima nel `PATH`.

## Strategia Applicata

La strategia e stata separare nettamente tre livelli:

1. **Dinamica SEA C++**
   - resta nel plugin;
   - le derivate `motor_angle_dot` e `motor_speed_dot` vengono esposte dal plugin
     come `Output` a `Stage::Dynamics`;
   - Python non implementa una dinamica SEA alternativa.

2. **Accelerazioni generalizzate**
   - restano calcolate con `compute_udot_bypass()`;
   - non si usa `realizeAcceleration()`;
   - questo evita il path nativo che aveva prodotto access violation.

3. **Integrazione Python**
   - Python integra nel tempo `q`, `qdot`, `motor_angle`, `motor_speed`;
   - le derivate SEA provengono dal plugin C++;
   - sono stati aggiunti substep adattivi e ripristino dello stato se un tentativo
     numerico fallisce.

## Modifiche Implementate

### `simulation_runner.py`

File:

```text
simulation_runner.py
```

Modifiche principali:

- aggiunto controllo esplicito:

```python
sea_forward_mode in {"plugin", "ideal_torque"}
```

- in modalita `plugin` non viene piu chiamato:

```python
_update_sea_motor_state(state, tau_sea_cmd)
```

quindi il runner non forza piu:

```text
motor_angle = q + tau_ref/K
motor_speed = qdot
```

- aggiunta lettura delle derivate SEA tramite output del componente:

```python
component.getOutput("motor_angle_dot")
component.getOutput("motor_speed_dot")
```

- aggiunto avanzamento plugin con substep:

```python
sea_motor_substeps = 20
sea_motor_max_substeps = 160
```

- aggiunto retry automatico:

```text
20 -> 40 -> 80 -> 160 substeps
```

- aggiunto snapshot/ripristino dello stato OpenSim prima di un tentativo di
  integrazione del passo;
- se il passo resta non finito anche con il massimo numero di substep, il runner:
  - salva gli output parziali;
  - scrive un file di stato;
  - solleva errore verso `main.py`;
  - impedisce che `--plot` o `--validate` vengano eseguiti su output incompleti.

Nuovo file generato per ogni run:

```text
results/.../sim_output_run_status.txt
```

Esempio run riuscito:

```text
status=complete
t=4.4
step=14
```

Esempio run fallito:

```text
status=failed
t=6.75
step=249
error_type=FloatingPointError
```

### `config.py`

File:

```text
config.py
```

Modifiche rilevanti:

```python
sea_forward_mode: str = "plugin"
sea_motor_substeps: int = 20
sea_motor_max_substeps: int = 160
save_sea_derivatives: bool = True
```

Significato:

- `plugin`: modalita validabile, non tautologica;
- `ideal_torque`: baseline diagnostica legacy;
- i substep servono per la dinamica SEA/plugin stiff;
- `save_sea_derivatives` abilita l'output diagnostico delle derivate lette dal
  plugin.

### `prosthesis_controller.py`

File:

```text
prosthesis_controller.py
```

Modifica minima:

- rimosso un commento operativo che citava `realizeAcceleration()`;
- mantenuta invariata la legge di controllo high-level;
- chiarito che il controllo deve essere scritto prima che gli output plugin a
  `Stage::Dynamics` leggano `getControl(state)`.

### `validation/validate_sim_results.py`

File:

```text
validation/validate_sim_results.py
```

Modifiche:

- aggiunta lettura di:

```text
sim_output_run_status.txt
```

- se il run status e `failed`, il validator produce `FAIL`;
- il validator ora controlla il tracking di tutte le coordinate disponibili,
  non solo:
  - coordinate protesiche;
  - traslazioni pelvis.

Questo ha rivelato un problema importante: alcune coordinate biologiche del
piede destro divergono gia nel run corto, in particolare:

```text
ankle_angle_r
subtalar_angle_r
mtp_angle_r
```

Questa e una diagnosi nuova e distinta dalla tautologia SEA.

### Plugin C++

La sessione precedente aveva gia introdotto nel plugin:

```text
motor_angle_dot
motor_speed_dot
tau_input
```

come `Output` leggibili dal runner.

La DLL corrente in `plugins/` risulta:

```text
plugins/SEA_Plugin_BlackBox_mCMC_impedence.dll
```

dimensione:

```text
196096 bytes
```

ultimo aggiornamento:

```text
2026-04-16 10:54:39
```

Nota: il plugin continua a stampare messaggi debug da `Plugin_interface.cpp`:

```text
[DEBUG WIN] >>> SYSTEM ALERT: IL PLUGIN E' STATO CARICATO IN MEMORIA.
```

Questa stampa non e stata rimossa perche non era nel perimetro richiesto delle
modifiche C++ minime.

## Verifiche Eseguite

### Compilazione Python

Comando:

```powershell
python -m py_compile config.py simulation_runner.py prosthesis_controller.py main.py validation\_debug_one_step.py
python -m py_compile validation\validate_sim_results.py simulation_runner.py config.py prosthesis_controller.py
```

Esito:

```text
OK
```

### Ricerca Chiamate Native Pericolose

Comando:

```powershell
rg -n "model\.realizeAcceleration\(|model\.computeStateVariableDerivatives\(|getStateVariableDerivativeValue\(" -g "*.py" .
```

Esito:

```text
nessun match operativo
```

Questo significa che il codice Python del simulatore non entra piu
volontariamente nel path nativo che aveva prodotto access violation.

### Smoke One-Step

Comando:

```powershell
conda run -n envCMC-like python -X faulthandler validation\_debug_one_step.py
```

Esito:

```text
OK
```

Risultati chiave:

```text
udot finite True
sea derivatives [0, -41131.935, 0, 50000]
```

Nessuna access violation osservata.

### Run Corto Plugin

Comando:

```powershell
conda run -n envCMC-like python -X faulthandler main.py --t-start 4.26 --t-end 4.40 --output-dir results\_validation_plugin_smoke_014_status --validate
```

Esito simulazione:

```text
complete
```

File stato:

```text
results/_validation_plugin_smoke_014_status/sim_output_run_status.txt
```

contenuto:

```text
status=complete
t=4.4
step=14
```

Esito SEA:

- `tau_ref - tau_spring` non e piu nullo a precisione macchina;
- `motor_angle` non e piu vincolato algebricamente a `q + tau_ref/K`;
- `motor_speed` non coincide piu con `qdot`;
- le derivate plugin sono finite.

Questo conferma che la tautologia SEA e stata rimossa nel path plugin.

### Run Lungo Plugin

Comando:

```powershell
conda run -n envCMC-like python -X faulthandler main.py --t-start 4.26 --t-end 6.90 --output-dir results\_validation_plugin_until_690_status --validate
```

Esito:

```text
failed controllato, nessun crash nativo
```

Il runner ha provato:

```text
20 -> 40 -> 80 -> 160 substeps
```

e poi si e fermato a:

```text
t=6.75
step=249
```

Errore controllato:

```text
Plugin forward step did not remain finite at t=6.7500 even with 160 substeps.
Last error: Non-finite accelerations during plugin substep 71/160:
['pelvis_tilt', 'pelvis_list', 'pelvis_rotation', 'pelvis_tx', 'pelvis_ty']
```

Il file:

```text
results/_validation_plugin_until_690_status/sim_output_run_status.txt
```

contiene:

```text
status=failed
t=6.75
step=249
```

Il validator non viene piu eseguito automaticamente da `main.py` dopo un run
fallito.

## Diagnosi Numerica Attuale

Il problema residuo non e piu la tautologia SEA e non e piu un accesso nativo
non controllato alla memoria.

Il problema residuo e una divergenza dinamica/numerica del modello.

Metriche osservate sul run parziale fino a `t=6.75`:

```text
pros_ankle_angle_qdot last = -110.905 rad/s
SEA_Knee_tau_motor last   = 500 Nm
SEA_Ankle_tau_motor last  = 500 Nm
SEA_Knee control max |u|  = 1.0
tau_target_norm last      = 697203
tau_reserve_norm last     = 245408
residual_norm last        = 512899
```

Questo indica:

- saturazione del motore SEA;
- richiesta dinamica enorme;
- crescita non fisica di alcune coordinate;
- impossibilita del recruitment muscolo/reserve di chiudere il target.

Il validator piu severo mostra inoltre divergenza precoce anche sul lato
biologico destro:

```text
ankle_angle_r
subtalar_angle_r
mtp_angle_r
```

gia nel run corto 4.26-4.40.

Questo sposta la prossima analisi su:

- stabilita del tracking biologico;
- trattamento di coordinate non muscolo-controllabili o debolmente controllate;
- integrazione semi-esplicita;
- mapping coordinate/mobility;
- reserve e bounds del lato destro;
- eventuale necessita di vincoli o tracking piu robusto per piede destro.

## Diagnosi DLL `0xc06d007f`

Controlli eseguiti:

```powershell
conda run -n envCMC-like python -c "import opensim,sys,os; ..."
```

Output rilevante:

```text
python  = C:\Users\tomma\anaconda3\envs\envCMC-like\python.exe
opensim = C:\OpenSim-mCMC\sdk\Python\opensim\__init__.py
PATH    = C:\OpenSim-mCMC\bin;...
```

Il plugin e compilato contro:

```text
C:/OpenSim-mCMC
```

Sono pero presenti anche:

```text
C:\OpenSim 4.1\bin
C:\OpenSim 4.5\bin
```

Raccomandazione operativa temporanea:

```powershell
conda run -n envCMC-like python -X faulthandler main.py --t-start 4.26 --t-end 4.40 --output-dir results\_safe_smoke --validate
```

Da evitare se l'ambiente non e chiaramente attivo:

```powershell
python main.py
```

perche potrebbe caricare una combinazione sbagliata di DLL OpenSim/Simbody.

## File Modificati

Nel simulatore:

```text
config.py
simulation_runner.py
prosthesis_controller.py
validation/validate_sim_results.py
```

Gia presenti o creati in precedenza nella linea di lavoro:

```text
main.py
output.py
validation/_debug_one_step.py
validation/validate_sim_results.py
```

Output diagnostici creati:

```text
results/_validation_plugin_smoke_014_status/
results/_validation_plugin_until_690_status/
reports/user/2026-04-16_validazione_simulatore.md
```

## Stato Attuale

Stato positivo:

- il path Python non usa piu le chiamate OpenSim native osservate come pericolose;
- lo smoke one-step e il run corto non generano access violation;
- la tautologia SEA e rimossa nel path plugin;
- il runner marca chiaramente i risultati parziali come `failed`;
- `main.py --validate` non procede piu dopo una simulazione fallita;
- il validator e piu severo e intercetta divergenze dinamiche.

Stato ancora aperto:

- il run lungo diverge a circa `t=6.75 s`;
- il lato biologico destro mostra instabilita precoce su piede/caviglia;
- `0xc06d007f` va trattato come possibile mismatch DLL/runtime e diagnosticato
  separatamente con un check ambiente;
- il plugin stampa ancora messaggi debug da `Plugin_interface.cpp`;
- la DLL corrente resta legata a `C:/OpenSim-mCMC`, quindi ogni lancio deve
  usare lo stesso runtime.

## Prossimi Passi Consigliati

1. Creare uno script diagnostico leggero, ad esempio:

```text
validation/check_runtime_environment.py
```

che controlli:

- Python eseguibile;
- path del modulo `opensim`;
- prime voci del `PATH`;
- presenza di `C:\OpenSim-mCMC\bin` prima di `C:\OpenSim 4.1` e `C:\OpenSim 4.5`;
- dimensione e timestamp della DLL plugin;
- eventuali DLL OpenSim duplicate nel processo.

2. Analizzare la divergenza biologica del lato destro prima di continuare sulla
   validazione SEA completa.

3. Non modificare ancora il plugin C++ finche non e chiaro se il collo di bottiglia
   e:

- tracking biologico;
- static optimization;
- integrazione;
- coordinate/mobility mapping;
- o saturazione SEA.

4. Mantenere per ora i run di test corti:

```powershell
conda run -n envCMC-like python -X faulthandler main.py --t-start 4.26 --t-end 4.40 --output-dir results\_safe_smoke --validate
```

5. Usare run lunghi solo per diagnosi controllata, aspettandosi possibile:

```text
status=failed
```

finche la divergenza dinamica non viene risolta.

