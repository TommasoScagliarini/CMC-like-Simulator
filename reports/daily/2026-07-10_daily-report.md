# Daily Report - 2026-07-10

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidato

- `reports/user/2026-07-10_reward_contact_support_ledger_h1_h2.md`
- `reports/user/2026-07-10_validazione_porting_warm_start_actor.md`

## Problema

La reward pagava ripetutamente il carico durante una stance compressiva che
terminava prima del TO. Inoltre il guard a `22 mm` interrompeva l'oracolo nella
terza stance, pur dopo due cicli validi, e la reward usava talvolta la fase
grezza del detector invece dello stato FSM accettato.

## Soluzione

E' stato introdotto un ledger contact-support:

- credito denso limitato a `0.04 BW*s` e provvisorio;
- conferma e bonus solo su TO FSM valido nella finestra `0.79-1.26 s`;
- qualita' media di penetrazione durante stance;
- clawback integrale su terminale, timeout o ciclo fallito;
- fase stance/swing derivata prioritariamente dallo stato FSM accettato.

La reward e le soglie ex-novo sono state congelate a:

```text
soft penetration = 15 mm
hard termination = 25 mm
penetration weight = 0.05
```

I default globali e i vecchi snapshot restano invariati.

## Validazione H0 e guard

- audit positivo: return `+51.287`, `2` cicli;
- matrice sintetica negativa: `9/9` PASS;
- H0 HS-TO-HS: `1` ciclo valido;
- full episode a 22 mm: `373/500` step, `2` cicli, terminale a `22.150 mm`;
- dopo allineamento FSM/reward: conflitti `118 -> 0`, return `+42.258`;
- full episode a 25 mm: `500/500` step, `3 HS / 3 TO / 2 cicli`;
- picco dinamico `22.9497 mm`, `8` step oltre 22 mm, `0` oltre 25 mm;
- return a 25 mm `+61.263`, discounted return `+5.711`.

Il replay dell'intero dataset AB06 prescritto ha inoltre mostrato, sul lato
sinistro applicato:

- IK preprocessata come nel simulatore: `0` campioni oltre 22/25 mm, massimo
  `21.740 mm`;
- IK grezza: `5` campioni oltre 22 mm e `1` oltre 25 mm, concentrati in due
  transitori ad alta frequenza; il filtro a 6 Hz li attenua.

## H1 fresh e H2

H1 fresh-policy da 10 iterazioni ha completato `40960` step, ma tutti i `1108`
episodi contabilizzati sono terminati per penetrazione. Il best return medio e'
`-2.104`.

H2 sul best ha prodotto `36` step, un HS, nessun TO e return `-2.121`. Il
credito denso provvisorio e' stato recuperato integralmente: il loophole e'
chiuso, ma l'esplorazione fresh non raggiunge il TO.

## Validazione porting warm-start

Il transplant actor imitativo `31 -> 39` feature e' stato validato prima di
qualsiasi update PPO. Sono state copiate per nome `29` feature, azzerate le `8`
feature FSM nuove e neutralizzate le `2` feature del gait clock sonoro,
disabilitato nel target.

Il trainer a zero iterazioni ha verificato che learner, EnvRunner locale,
EnvRunner remoto e modulo salvato condividano il digest actor
`b6c7da24e0014edfddc282a2b8187cd58f14691537fb62f7d923f71d98b9ed8a`,
con differenza massima `0.0`. I sei tensori del critic target sono rimasti
invariati, anch'essi con differenza `0.0`, e non e' stato importato alcuno stato
optimizer dal source.

Il gate finale ha dato `PASS_WITH_WARNINGS`, zero failure:

- equivalenza su `4096` input sintetici: errore massimo `0.0`;
- equivalenza su un input OpenSim reale: errore massimo logits `1.1920929e-7`;
- azione registrata vs media actor: errore massimo `5.9604645e-8`;
- manifest runtime target coerente: `39/84` feature actor/full;
- profilo fisico online-GRF source/target identico per SHA-256.

Le warning descrivono shift noti: detector HS/TO separato, clock sonoro
disabilitato, start offset diverso e slew limiter solo target. Lo step reale ha
inoltre mostrato una componente action fuori range (`|raw|max=1.7813`, applicata
a `1.0`): e' un segnale da misurare nel rollout completo, non un errore di
porting.

E' stato infine chiuso un rischio nei rollout storici: un checkpoint `31/80`
non puo' piu' essere valutato silenziosamente nell'ambiente corrente `43/88`.
`rollout_eval.py` ora interrompe con un errore esplicito di schema.

## File modificati o aggiornati

- `Trajectory Generator/baseline_MLP/reward_function.py`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
- `Trajectory Generator/baseline_MLP/training_config.py`
- `Trajectory Generator/baseline_MLP/README.md`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `Trajectory Generator/baseline_MLP/warm_start.py`
- `validation/prescribed_reward_probe.py`
- `validation/reward_audit_suite.py`
- `validation/test_reward_function.py`
- `validation/test_warm_start.py`
- `validation/test_rollout_eval.py`
- `validation/validate_training_config.py`
- `validation/validate_warm_start_port.py`

## Test e verifiche

- reward unit tests: `32/32` PASS;
- config smoke con baseline `15/25 mm`: PASS;
- audit sintetici: `9/9` PASS;
- audit positivo e probe H0: PASS;
- full episode a 25 mm: PASS;
- confronto trace 25 vs 30 mm: dinamica/eventi identici, cambiano solo i due
  campi reward attesi;
- H1 fresh: PASS tecnico;
- H2 fresh: FAIL comportamentale.
- warm-start/schema-rollout unit test: `6/6` PASS;
- trainer warm-start a zero iterazioni con EnvRunner remoto: PASS;
- gate porting warm-start: PASS_WITH_WARNINGS, zero failure;
- smoke target reale di un passo: PASS;
- guardia checkpoint storico `31/80` su env `43/88`: rifiuto atteso PASS.

## Chiarimento: rollout deterministico iniziale warm-start

Significa caricare `rl_module_initial_warm_start`, cioe' l'attore subito dopo
il transplant imitativo e prima del primo gradiente PPO, ed eseguire un episodio
con esplorazione disattivata, stesso start e stessa configurazione congelata.
Non e' training: misura il comportamento introdotto dal solo transplant e lo
separa dagli effetti delle dieci iterazioni PPO.

## TODO aperti e propagati

- [ ] Eseguire il rollout deterministico completo di
      `rl_module_initial_warm_start` sulla baseline `15/25 mm`.
- [ ] Eseguire H1 warm-start da 10 iterazioni con stesso seed e budget di H1
      fresh.
- [ ] Eseguire H2 sul best warm-start.
- [ ] Confrontare fresh e warm-start su return, durata, HS/TO/cicli,
      penetrazione, clawback, clipping e diagnostica SEA.
- [ ] Sbloccare training piu' lunghi solo se H2 warm-start raggiunge almeno un
      TO valido senza terminazione precoce.
- [ ] Spiegare o eliminare il TO precoce rifiutato nella seconda stance
      dell'oracolo multi-ciclo.
- [ ] Tenere la memoria in coda finche' non emerge un limite sequenziale reale
      dopo la validazione warm-start.
