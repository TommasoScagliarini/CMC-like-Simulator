# 2026-05-29 - Env RL traiettorie CMC-like e overview per LLM

## Problema

Serviva adattare un file environment RL OpenSim esterno al simulatore
CMC-like del repository.

L'obiettivo non era far comandare alla rete neurale muscoli, reserve o coppie
dirette, ma farle generare segmenti di traiettoria cinematica di riferimento
per i due SEA protesici:

```text
pros_knee_angle
pros_ankle_angle
```

Questi segmenti devono poi essere interpolati e inseguiti dall'outer loop dei
SEA, mentre tutte le coordinate non protesiche devono continuare a seguire i
dati cinematici presenti nei file `kin_ref`.

Dopo la prima bozza dell'env sono emerse issue architetturali:

- accoppiamento troppo forte dell'env a metodi privati di `SimulationRunner`;
- reset invasivo e duplicazione manuale dell'inizializzazione del runner;
- default `rebuild_model_on_reset=True` troppo costoso per RL/PPO;
- fallback Eulero locale fragile e non coerente col path validato;
- gestione errori troppo silenziosa;
- reward troppo centrato sul tracking del target generato dalla policy;
- necessita' di un documento sintetico per dare contesto a un altro LLM.

## Soluzione

Sono stati introdotti due livelli di soluzione.

Primo: un nuovo environment Gymnasium `osim_trj_cmc_like.py`, in cui la policy
produce un segmento di traiettoria per knee/ankle protesici. Il riferimento
protesico viene sostituito tramite un wrapper cinematico, mentre il resto delle
coordinate continua a provenire dal `kin_ref` originale.

Secondo: `SimulationRunner` e' stato esteso con una piccola API pubblica per
uso step-wise/RL:

```text
reset_to_time(t)
step_until(t_stop, record=True)
save_results()
reset_outputs()
state
current_time
last_step_info
```

In questo modo l'env non chiama piu' direttamente `_compute_controls_for_window`,
`_integrate_evaluate`, `_set_state`, `_prosthesis_ctrl`, `_so` o altri dettagli
interni del runner.

E' stato inoltre creato `LLM_SIMULATOR_OVERVIEW.md`, un documento compatto per
LLM che spiega cosa fa il simulatore, come viene controllata la parte biologica
e come funziona la struttura di controllo protesica.

## Strategia

La strategia e' stata conservativa:

- non modificare il plugin C++;
- non cambiare la semantica del comando SEA;
- mantenere il path validato `sea_forward_mode="plugin"` e
  `integration_scheme="rk4_bypass"`;
- isolare l'interfaccia RL in un file separato;
- promuovere solo le primitive minime necessarie nel runner;
- rendere il reset riusabile di default per non ricostruire OpenSim a ogni
  episodio;
- far fallire esplicitamente l'env se viene usato con path numerici non
  validati;
- rendere la gestione errori configurabile: `fail_fast=True` per sviluppo,
  `fail_fast=False` per convertire l'errore in `truncated=True` con traceback
  completo in `info`.

Per la traiettoria policy e' stata usata un'interpolazione con
`CubicHermiteSpline` quando sono disponibili derivate, impostando la velocita'
iniziale del segmento uguale alla velocita' del target corrente. Questo riduce
le discontinuita' tra segmenti rispetto a una semplice interpolazione per
posizione.

## File modificati

```text
simulation_runner.py
```

Modifiche principali:

- aggiunta API pubblica step-wise/RL;
- aggiunta inizializzazione pubblica `reset_to_time`;
- aggiunta avanzamento pubblico `step_until`;
- aggiunta `reset_outputs` per pulire il recorder tra episodi riusati;
- mantenuto il loop standard `run()` invariato come entry point batch.

## File creati

```text
osim_trj_cmc_like.py
```

Nuovo adapter Gymnasium:

- action space = knot di traiettoria per i due DOF protesici;
- modalita' azione: `delta`, `absolute`, `raw`;
- sostituzione solo del riferimento protesico;
- coordinate biologiche sempre delegate al `kin_ref`;
- reward con tracking del target policy, riferimento protesico sano/IK,
  tracking biologico, effort e smoothness;
- reset riusabile di default (`rebuild_model_on_reset=False`);
- validazione del path runtime `plugin + rk4_bypass`.

```text
LLM_SIMULATOR_OVERVIEW.md
```

Documento per LLM:

- scopo del simulatore;
- flusso generale CMC-like;
- distinzione parte biologica / parte protesica;
- dinamica SEA low-level;
- controller protesico cascade;
- ruolo dell'env RL di traiettoria;
- invarianti e assunzioni da non violare.

## Test e verifiche eseguite

Verifica sintattica:

```powershell
python -m py_compile simulation_runner.py osim_trj_cmc_like.py
```

Esito: completata senza errori.

Verifiche statiche:

- controllato che `osim_trj_cmc_like.py` non usi piu' metodi privati del runner
  tramite pattern `runner._` / `self.runner._`;
- controllato che il fallback locale `_advance_euler_without_plugin` sia stato
  rimosso;
- controllato lo stato Git:

```text
 M simulation_runner.py
?? LLM_SIMULATOR_OVERVIEW.md
?? osim_trj_cmc_like.py
```

Non e' stata eseguita una smoke dinamica OpenSim completa perche' carica
modello, plugin e GRF e puo' richiedere tempo significativo.

## TODO aperti

- Eseguire una smoke dinamica breve dell'env RL, ad esempio un episodio da
  `0.02-0.05 s`, per verificare caricamento modello/plugin e un singolo
  `env.step()`.
- Misurare il costo di `reset()` con `rebuild_model_on_reset=False` e con
  `True`, per stimare la fattibilita' PPO.
- Validare se il reset riusabile non lascia stato residuo indesiderato nel
  plugin SEA o negli ExternalLoads.
- Tarare i pesi del reward dopo i primi rollout reali.
- Decidere se inserire `LLM_SIMULATOR_OVERVIEW.md` anche nel flusso
  `start_day`/`CONTEXT.md` come riferimento stabile per futuri agenti.

