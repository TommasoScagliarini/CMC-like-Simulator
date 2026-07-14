# Validazione porting warm-start actor imitativo -> ex-novo

Data: 2026-07-10

Instruction check token: CMC_AGENT_OK_2026

## Problema

Prima di avviare H1 warm-start era necessario dimostrare che il porting della
rete imitativa nel nuovo actor ex-novo fosse semanticamente corretto. Il source
checkpoint usa `31` feature attore, mentre il target ne usa `39`; inoltre il
target disabilita il gait clock prescritto e aggiunge otto feature FSM.

Un semplice `load_state_dict` non era sufficiente: avrebbe potuto associare i
pesi alle colonne sbagliate, introdurre un bias costante dal clock disabilitato,
modificare il critic oppure lasciare gli EnvRunner con i pesi random iniziali.

## Strategia

La validazione e' stata eseguita prima di qualsiasi update PPO:

1. controllo di classe RLModule, architettura, action space, versioni Ray e
   contratto di configurazione;
2. transplant per nome delle feature comuni;
3. azzeramento delle otto colonne FSM nuove e delle due colonne del gait clock
   non disponibile nel target;
4. copia delle restanti parti dell'actor e conservazione del critic target
   appena inizializzato;
5. sincronizzazione esplicita learner -> EnvRunner prima del primo sample;
6. confronto tensoriale, confronto funzionale sintetico e confronto su una
   vera osservazione OpenSim target;
7. controllo delle differenze di dominio GRF e post-processing.

E' stato inoltre eseguito un negative control sul vecchio snapshot warm-start:
il gate lo ha rifiutato perche' manteneva le colonne del gait clock e non
conteneva le prove d'integrazione richieste.

## Risultato

Esito finale: `PASS_WITH_WARNINGS`, `port_validated=true`.

- source actor: `31` feature;
- target actor: `39` feature;
- feature copiate: `29`;
- feature FSM target inizialmente inerti: `8`;
- feature gait clock condivise ma neutralizzate: `2`;
- digest actor learner, EnvRunner locale, EnvRunner remoto e modulo salvato:
  `b6c7da24e0014edfddc282a2b8187cd58f14691537fb62f7d923f71d98b9ed8a`;
- differenza tensoriale massima dopo save/reload: `0.0`;
- critic target: tutti i `6` tensori `vf`/`vf_encoder` invariati, differenza
  massima `0.0`, digest prima/dopo
  `8d6042483e1fcde1b8998adea8f58e3e91ff033936d056a0df95fd1bd498de50`;
- optimizer source caricato: no;
- equivalenza su `4096` input sintetici allineati: errore massimo logits e
  azione deterministica `0.0`;
- equivalenza sulla vera osservazione target: errore massimo logits
  `1.1920929e-7`;
- azione registrata vs media dell'actor target: errore massimo `5.9604645e-8`;
- manifest runtime target: `39` feature actor e `84` feature complete, coerente
  col checkpoint.

Lo smoke OpenSim di un solo step ha completato senza terminazione. L'azione raw
ha raggiunto `1.7813` ed e' stata limitata a `1.0`; una delle due componenti era
fuori range. Questo non invalida il porting, ma rende necessario il rollout
deterministico completo prima di H1 per misurare clipping, penetrazione ed
eventi lungo tutto l'episodio.

## Warning di dominio

Le warning residue non sono mismatch di rete:

- il source non aveva il detector HS/TO separato, presente nel target;
- il source usava il gait clock sonoro, disabilitato nel target e neutralizzato
  azzerandone le colonne;
- start offset `1.0 s` nel source contro `1.9568709838 s` nel target;
- slew limiter assente nel source e presente nel target (`2.5/2.0 rad/s`);
- il vecchio profilo GRF source non e' nel worktree ed e' stato recuperato da
  Git; il contenuto fisico e' comunque identico al profilo target, con SHA-256
  `09e04ab94954703d74acc3a80b24ecefcc07d3fc918c03b9e9df8116a6c1a2b0`.

## Compatibilita' dei rollout storici

Il checkpoint imitativo storico dichiara `31/80` feature actor/full, ma
l'ambiente imitativo corrente produce `43/88` feature dopo l'aggiunta dei
target imitativi e della FSM. Prima della correzione, `rollout_eval.py` tagliava
silenziosamente il prefisso e quindi poteva usare semantiche errate.

E' stato aggiunto un controllo fail-fast: il checkpoint storico viene ora
rifiutato con il mismatch `31/80 -> 43/88`. Il confronto del porting non usa
quel rollout invalido; ricostruisce invece il vettore source per nome a partire
dalle osservazioni target.

## File modificati

- `Trajectory Generator/baseline_MLP/warm_start.py`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `validation/validate_warm_start_port.py`
- `validation/test_warm_start.py`
- `validation/test_rollout_eval.py`

## Artefatti

- target iniziale validato:
  `validation/warm_start_port_runs/2026-07-10_validated/trainer_zero_iter_v4/rl_module_initial_warm_start/`;
- report transplant:
  `validation/warm_start_port_runs/2026-07-10_validated/trainer_zero_iter_v4/actor_transplant_report.json`;
- gate finale JSON/Markdown:
  `validation/warm_start_port_runs/2026-07-10_validated/final_v4/warm_start_port_validation.{json,md}`;
- trace reale:
  `validation/warm_start_port_runs/2026-07-10_validated/target_env_smoke_v4/rollout_policy_trace.json`;
- negative control:
  `validation/warm_start_port_runs/2026-07-10_negative_old_smoke/`.

## Test e verifiche

- `py_compile`: PASS;
- warm-start e schema rollout: `6/6` unit test PASS;
- reward regression: `32/32` PASS;
- training-config smoke: PASS;
- trainer RLlib con `0` iterazioni PPO e un EnvRunner remoto: PASS;
- gate porting finale: PASS_WITH_WARNINGS, zero failure;
- smoke target reale dopo la guardia schema: PASS;
- smoke source moderno con checkpoint storico: rifiuto atteso per mismatch di
  schema.

## Conclusione e TODO

Il porting della rete e' validato. Non e' ancora validato il comportamento su
un episodio completo e non e' stato eseguito training PPO.

- [ ] Eseguire il rollout deterministico completo del modulo
      `rl_module_initial_warm_start` con baseline congelata `15/25 mm`.
- [ ] Avviare H1 warm-start da `10` iterazioni solo dopo il controllo del
      rollout iniziale.
- [ ] Eseguire H2 sul best warm-start e confrontarlo con H1/H2 fresh.
