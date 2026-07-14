# Validazione reward prescribed e configurazione ex-novo

Data: 2026-07-02

## Problema

Il rollout del training ex-novo da 100 iterazioni con osservazione ricca non ha
chiuso la sequenza desiderata:

```text
HS -> TO -> HS
```

Il comportamento osservato e' stato:

```text
HS protesico -> TO protesico -> swing prolungato -> phase_timeout:swing
```

Nel rollout migliore la policy teneva la gamba in una configurazione fortemente
flessa dopo il TO, senza ricostruire il contatto necessario al secondo HS. Il
problema non sembrava dovuto alla FSM o al detector GRF in se': la FSM
rilevava correttamente il primo HS e il TO, ma la reward continuava a rendere
redditizio il fallimento parziale.

In parallelo e' emersa una seconda esigenza: testare la reward su una camminata
prescribed/IK, cioe' su dati che rappresentano una locomozione gia' corretta,
per capire se la reward premia davvero un comportamento valido prima di
lanciare nuovi training lunghi.

## Diagnosi

Il rollout 100iter analizzato e':

```text
Trajectory Generator/runs/rollout/
MLP_ExNovo_rollout_07-01-2026_ledger_clawback_richobs_100iter_07-01-2026_richobs100_best_rollout_07-02-2026
```

Esito sintetico:

```text
steps: 141
episode_return: 17.4305
terminated: true
end_reason: phase_timeout:swing
left HS: 12.990 s
left TO: 13.099 s
second left HS: assente
HS -> TO: circa 0.109 s
```

La durata `HS -> TO` era molto piu' corta della stance nominale attesa. Dopo il
TO non veniva ricreato contatto utile: la gamba restava in swing e il rollout
terminava per timeout.

La replay/counterfactual reward sul trace fallito ha mostrato che il termine di
tracking del riferimento servito era uno dei principali incentivi al blocco
della gamba:

```text
config salvata:                         +17.431
blend_tracking = 0:                      -7.992
policy_action_clip_weight = 0.25:       +11.451
blend_tracking = 0 + clip_weight = 0.25: -8.026
phase_timeout_penalty_weight = 1.0:      -4.386
phase_clawback_penalty_weight = 5.0:    +16.031
```

Interpretazione: la policy poteva ottenere reward positiva inseguendo il
riferimento servito anche mentre il mezzo ciclo falliva. Il problema principale
non era quindi solo il ledger/clawback, ma il fatto che il tracking del comando
servito poteva premiare una configurazione biomeccanicamente inutile.

## Soluzione

Sono stati fatti tre interventi.

1. La fase morfologica diagnostica e' stata agganciata alla FSM protesica quando
   disponibile, tramite `fsm_morphology_phase`. Questo evita di basare il
   corridor su una fase online fragile prima del primo ciclo completo.

2. La configurazione ex-novo e' stata resa piu' severa contro il failure mode
   osservato:

```yaml
blend_tracking: 0.0
blend_contact_load: 0.35
blend_phase_regular: 0.25
blend_phase_event_progress: 1.00
blend_landing_window_contact: 0.25

phase_min_stance_duration_s: 0.30
phase_min_swing_duration_s: 0.25
phase_landing_window_start_s: 0.35
phase_landing_window_end_s: 0.85
phase_swing_timeout_s: 0.75
phase_swing_hard_timeout_s: 1.10
phase_timeout_penalty_weight: 0.50

policy_action_clip_weight: 0.25

morphology_weight: 0.0
morphology_std_multiplier_knee: 1.6
morphology_std_multiplier_ankle: 0.6
morphology_margin_knee_deg: 7.5
morphology_margin_ankle_deg: 7.5
```

Il corridor morfologico resta disattivato nella reward (`morphology_weight: 0.0`)
perche' la priorita' e' validare prima la FSM e la reward di fase.

3. E' stato aggiunto uno script di validazione prescribed:

```text
validation/prescribed_reward_probe.py
```

Lo script costruisce un env sulla finestra prescribed/IK, mantiene attivi online
GRF detector e FSM, ma comanda la protesi con zero delta rispetto alla
traiettoria prescribed. In questo modo la reward viene valutata su una camminata
nota, non su una policy addestrata.

## Strategia

La strategia e' stata separare tre domande:

- la FSM riconosce correttamente `HS` e `TO`?
- la reward punisce davvero il mezzo ciclo `HS -> TO -> timeout`?
- la stessa reward premia una camminata prescribed corretta?

Per evitare di sovraccaricare il PC mentre un training smoke da 10 iterazioni
era attivo, la validazione prescribed e' stata provata inizialmente su finestre
corte, sufficienti a validare esecuzione, output e prime diagnostiche.

## File modificati

- `Trajectory Generator/baseline_MLP/reward_function.py`
  - aggiunta fase morfologica derivata dalla FSM;
  - aggiunti diagnostici `fsm_morphology_phase`,
    `morphology_phase_source_id`, `morphology_phase_fsm_available`.

- `Trajectory Generator/baseline_MLP/tb_logging.py`
  - aggiunto logging dei nuovi diagnostici morfologia/FSM.

- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
  - disattivato il tracking servito nella reward ex-novo;
  - aggiunta penalita' per action clipping;
  - aggiornati pesi di contatto, fase e landing window;
  - mantenuto il corridor con peso nullo ma parametri diagnostici aggiornati.

- `validation/test_reward_function.py`
  - aggiunti/aggiornati test sui nuovi termini reward e sulla fase morfologica
    derivata dalla FSM.

- `validation/validate_training_config.py`
  - aggiornate le attese sulla configurazione ex-novo corrente.

- `validation/prescribed_reward_probe.py`
  - nuovo script per valutare la reward su dati prescribed/IK.

## Test e verifiche eseguite

Verifiche statiche e unit test:

```text
python -m py_compile validation/prescribed_reward_probe.py
validation/prescribed_reward_probe.py --help
validation/test_reward_function.py
validation/validate_training_config.py
validation/test_phase_fsm_prescribed_env.py
git diff --check
```

Sono stati generati plot diagnostici secondo lo standard della cartella `plot/`,
con output in:

```text
plot/07_02_2026_1
```

E' stato validato il comando corretto per visualizzare un rollout MLP:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python visualize.py --mlp --speed 0.5
```

Lo script prescribed e' stato testato con due smoke test reali OpenSim.

Smoke test 1:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/prescribed_reward_probe.py \
  --start-time 12.99 --end-time 13.05 \
  --output-dir validation/prescribed_reward_probe_runs/_validation_smoke_12p99_13p05
```

Risultato:

```text
ok: true
steps: 6
return: -1.12093
reward_mean: -0.186822
end_reason: episode_time_limit
final FSM: STANCE_AFTER_HS
valid HS: 1
valid TO: 0
valid cycles: 0
```

Smoke test 2:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/prescribed_reward_probe.py \
  --start-time 12.99 --end-time 13.12 \
  --output-dir validation/prescribed_reward_probe_runs/_validation_smoke_12p99_13p12
```

Risultato:

```text
ok: true
steps: 13
episode_return: -2.75134
reward_mean: -0.211641
end_reason: episode_time_limit
final FSM: STANCE_AFTER_HS
valid HS: 1
valid TO: 0
valid cycles: 0
policy_action_clip_loss: 0
prosthetic_joint_range_loss: 0
```

Gli output prodotti sono:

```text
summary.json
summary.md
trace.csv
online_events.csv
```

## Risultato importante

La validazione prescribed ha gia' evidenziato un problema potenziale: anche su
traiettoria prescribed la reward early-stance risulta negativa perche'
`grf_slip_loss` arriva alla saturazione:

```text
grf_slip_loss: 25.0
grf_slip_weight: 0.02
contributo: circa -0.5 per step
```

Questo significa che una camminata prescribed, che dovrebbe essere un riferimento
corretto, viene punita in modo consistente dal termine di slip GRF. Prima di
interpretare nuovi training lunghi conviene quindi ispezionare quel termine:
potrebbe essere troppo severo, mal scalato, oppure andrebbe gate-ato in base al
contesto GRF/prescribed.

## Stato finale

Lo script `validation/prescribed_reward_probe.py` e' stato creato e validato con
esecuzioni OpenSim reali su finestre corte. Non e' stata ancora eseguita la
finestra completa `12.99 -> 17.99` per evitare di sovraccaricare la macchina
mentre il training era ancora attivo.

La configurazione corrente riduce l'incentivo a stabilizzare la gamba nella
configurazione fallita osservata, ma il test prescribed indica che il termine
`grf_slip_loss` deve essere controllato prima di considerare la reward
completamente validata.

## Comandi utili

Test prescribed standard:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/prescribed_reward_probe.py \
  --start-time 12.99 --end-time 17.99
```

Test prescribed piu' lungo:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/prescribed_reward_probe.py \
  --start-time 11.99 --end-time 21.0
```

Con salvataggio STO:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python validation/prescribed_reward_probe.py \
  --start-time 12.99 --end-time 17.99 \
  --record-sim-outputs
```

## TODO

- Eseguire la probe completa `12.99 -> 17.99` quando il training non sta usando
  risorse pesanti.
- Ispezionare `grf_slip_loss` su prescribed e decidere se ridurre peso, cambiare
  scala o gate-are il termine.
- Dopo la correzione/validazione della reward su prescribed, rilanciare training
  ex-novo e verificare nel rollout:
  - presenza di almeno un ciclo `HS -> TO -> HS`;
  - `phase_valid_cycle_count > 0`;
  - assenza o riduzione di `phase_timeout:swing`;
  - assenza di clipping persistente dell'azione;
  - durata `HS -> TO` compatibile con una stance non degenerata.
