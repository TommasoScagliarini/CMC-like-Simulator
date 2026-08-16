# Detector V20 — FSM binaria HS/TO e gate DEV02/04

## Esito

La FSM binaria è stata implementata e si comporta correttamente come macchina
causale: il debounce è esattamente 5 ms, il timestamp dell'evento resta
all'onset, il processamento scalare a 1 ms e quello batch da 10 ms sono
identici, e gli input malformati falliscono senza avanzare lo stato.

La combinazione scientifica **geometria V19 + FSM V20**, però, non supera il
confronto con l'oracle biomeccanico. Il receipt è:

`FAIL_V20_BINARY_FSM_DEVELOPMENT_TERMINAL_TRIAL08_CLOSED`

Il risultato chiarisce la distinzione che stavamo cercando:

- il segnale V19 è realmente binario, deterministico e privo della gradualità
  delle sfere;
- la logica software interpreta coerentemente quel segnale;
- ma il piano geometrico virtuale viene intersecato dal piede troppo presto
  rispetto al contatto definito dalla GRF prescritta a 20 N.

Gli HS V20 risultano in mediana circa **102–142 ms anticipati** nei diversi
plateau. Non si tratta quindi di un ritardo introdotto dalla FSM: il suo ritardo
di conferma è sempre 5 ms. Il disallineamento principale nasce dal significato
fisico del segnale geometrico grezzo. Alle velocità più alte compaiono inoltre
transizioni/cicli extra.

## Semantica implementata

I quattro stati grezzi sono separati dalla fase funzionale:

- `00` → `AIR`;
- `10` → `HEEL`;
- `11` → `BOTH`;
- `01` → `TOE`.

La FSM applica il debounce alla classe globale `heel OR toe`, non alla parola
binaria completa. Di conseguenza una normale sequenza `00 → 10 → 11` conserva
l'onset del primo contatto e non riavvia il timer.

- HS funzionale: primo passaggio stabile `AIR → qualsiasi contatto`;
- TO funzionale: primo passaggio stabile `qualsiasi contatto → AIR`;
- conferma: esattamente 5 ms dopo l'onset (`i → i+5`);
- consegna: una sola volta al boundary policy;
- al reset il campione `t0` inizializza lo stato senza emettere eventi;
- un episodio iniziato in stance può emettere un TO successivo marcato come
  stance parziale al reset.

Le transizioni regionali `HEEL/BOTH/TOE` restano diagnostiche e non possono
generare HS o TO aggiuntivi.

## Integrazione shadow

È stato aggiunto un percorso indipendente `binary_shadow`. La nuova FSM non
scrive mai in `_online_events`, non modifica la vecchia `ProstheticPhaseFSM` e
non entra in osservazioni, reward, azioni o dinamica.

Il default resta:

- `phase_fsm_input_mode=legacy_events`;
- `event_contract_id=legacy_events_v1`;
- `binary_phase_fsm_mode=disabled`.

La configurazione attiva `training_exnovo_cfg.yaml` non è stata modificata e
non è stato creato né avviato alcun training. Come già congelato in V19, il
profilo analogico a sfere e quello binario sono mutuamente esclusivi. Pertanto
un futuro confronto runtime bit-identico dovrà usare una baseline accoppiata con
lo stesso profilo analogico disabilitato; non va confuso con la configurazione
attiva attuale che contiene ancora il detector analogico legacy.

## Validazione

Sono state valutate 16 unità:

- trial development 02 e 04;
- quattro plateau per trial;
- consumo sequenziale 1 ms e batch sugli stessi campioni da 10 ms.

Risultati principali:

- parità scalare/batch: **PASS 2/2 trial**, inclusi eventi, transizioni,
  cancellazioni dei candidati, pending state e payload finale;
- unit test e regressioni: **61 PASS**;
- unità scientifiche: **0/16 PASS**;
- F1 di fase confermata: circa `0.847–0.905`, sotto `0.95`;
- IoU: circa `0.734–0.826`, sotto `0.90`;
- debounce osservato: sempre 5 ms;
- ritardo consegna: entro 10 ms;
- trial 08 non aperto;
- trial protetti 05/06 e reserve 03/07 non aperti;
- nessuna GRF è stata risogliata: sono state riutilizzate le ledger canoniche
  congelate 02/04.

Receipt no-clobber:

`validation/binary_phase_fsm_v20_development_receipt.json`

SHA-256 receipt:

`43fe41ba938b020f75604a5c21dbe433a12bc3e8233be47f98bc697d9ac41e2c`

## File modificati o aggiunti

- `Trajectory Generator/binary_phase_fsm.py`: nuova FSM binaria V20;
- `Trajectory Generator/osim_trj_cmc_like.py`: trasporto e diagnostica shadow
  separati dal percorso legacy;
- `Trajectory Generator/baseline_MLP/training_config.py`: schema dei parametri
  shadow;
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`: plumbing CLI/manifest,
  disabilitato di default;
- `Trajectory Generator/baseline_MLP/rollout_eval.py`: plumbing equivalente per
  replay diagnostici;
- `validation/test_binary_phase_fsm_v20.py`: test causali e fail-closed;
- `validation/test_binary_phase_fsm_env_v20.py`: test di isolamento runtime;
- `validation/validate_binary_phase_fsm_v20_development.py`: replay DEV02/04;
- `validation/binary_phase_fsm_v20_development_receipt.json`: evidenza finale.

Restano inoltre presenti le modifiche V19 non ancora committate relative alla
geometria binaria; non sono state alterate la GRF primaria online, la geometria
di contatto primaria, `online_grf.py`, il plugin C++ o la semantica SEA.

## Decisione e prossimo confronto

Il ramo V20 è chiuso senza fallback o retuning e non può aprire trial 08. Prima
di qualunque nuova versione dobbiamo decidere esplicitamente quale contratto
fisico vogliamo per un sensore binario simulato: contatto puramente geometrico
al piano attuale, oppure una geometria/condizione osservabile che rappresenti
meglio l'inizio del carico reale. I risultati indicano che modificare ancora la
FSM non correggerebbe l'anticipo sistematico del segnale grezzo.
