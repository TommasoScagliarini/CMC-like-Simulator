# Reward contact-support con ledger e validazione H1/H2

Data report: 2026-07-10

Instruction check token: CMC_AGENT_OK_2026

## Problema

Il precedente H2 terminava prima del `toe_off` sinistro per
`grf_penetration`, ma chiudeva ancora con reward positiva. Il termine
`contact_load_score` pagava ripetutamente la compressione della stance e il
credito accumulato superava le penalita' terminali. Inoltre il guard a `17 mm`
non era fisicamente raggiungibile neppure dalla traiettoria zero-delta
prescritta nello stesso contratto dinamico del training.

Serviva distinguere due domande:

1. la reward assegna segno corretto a supporto completato e compressione
   fallita?
2. una policy PPO inizializzata casualmente riesce poi a raggiungere il `TO`?

## Soluzione

E' stato introdotto un ledger stateful per il supporto protesico:

- il carico e' un segnale debole di confidenza, saturo a `0.20 BW`;
- il credito denso si arresta dopo `0.04 BW*s` di evidenza;
- il credito resta provvisorio fino a un `TO` FSM valido;
- il `TO` riceve un bonus nella finestra di stance `0.79-1.26 s`, modulato
  dalla qualita' media della penetrazione durante la stance;
- terminazione, timeout o fallimento prima del `TO` recuperano integralmente
  il credito provvisorio.

I nuovi campi `RewardConfig` hanno default disabilitati, quindi vecchi YAML e
checkpoint mantengono la semantica precedente.

Il probe esatto `training_like` ha inoltre calibrato, solo per la configurazione
ex-novo:

```text
grf_penetration_penalty_threshold_m = 0.015
grf_penetration_termination_m       = 0.022
grf_penetration_weight              = 0.05
```

I default globali `0.012/0.017 m` non sono stati modificati.

## Strategia

La validazione e' stata eseguita in ordine causale:

1. test unitari e audit sintetici negativi;
2. audit positivo con gait prescritta;
3. H0 dall'esatto stato iniziale con contatto online applicato;
4. H1 da 10 iterazioni e policy casuale;
5. H2 deterministico registrato sul checkpoint best.

H1 e H2 sono stati autorizzati solo dopo il passaggio dei gate preliminari.

## Risultati preliminari

Audit positivo prescritto:

```text
status = PASS
valid HS / TO / cycle = 3 / 2 / 2
episode_return = +51.2874
reward_mean = +0.126636
max contact_support_to_score = 1.0
```

Tutti i nove scenari sintetici negativi passano, inclusi
`compressive_stance`, `missing_to`, `missing_second_hs` e
`fake_cycle_ankle_only`. Nel caso compressivo senza TO il ritorno e' negativo
e il credito provvisorio viene recuperato.

H0 `training_like`, zero-delta, finestra `13.94687-15.30 s`:

```text
steps = 136
valid HS / TO = 1 / 1
TO time from HS = 1.199 s
max penetration = 21.5625 mm
terminated = false
episode_return = +0.299094
max contact_support_to_score = 0.365704
```

Quindi una traiettoria fisicamente valida esiste nel contratto di training e
ha ritorno positivo.

### H0 esteso al secondo HS

La stessa prova e' stata estesa fino a `15.80 s`, senza cambiare reward,
contatto o soglie:

```text
steps = 186
valid HS / TO / cycle = 2 / 1 / 1
first cycle period = 1.466 s
first stance fraction = 0.8179
max penetration = 21.5625 mm
terminated = false
episode_return = +4.923501
```

Il contratto `training_like` raggiunge quindi una sequenza online completa:

```text
HS 13.946871 -> TO 15.145871 -> HS 15.412871
```

Dopo il secondo HS il detector osserva un TO a soli `0.123 s`. Il FSM lo
rifiuta per durata di stance inferiore al minimo `0.30 s`, incrementa
`invalid_event_count` e recupera il credito provvisorio della nuova stance.
Questa anomalia non invalida il primo ciclo, ma impedisce di considerare ancora
validata una camminata multi-ciclo stabile.

### H0 sull'intero episodio di training

Il probe e' stato infine esteso all'intero orizzonte richiesto di `5.0 s`:

```text
requested steps = 500
executed steps = 373
valid HS / TO / cycle = 3 / 2 / 2
invalid events = 1
max penetration = 22.1499 mm
terminated = true
end_reason = grf_penetration
episode_return = -2.742767
```

Il gate richiesto `valid_cycle_count >= 1` e' PASS con margine: vengono
completati due cicli. L'oracolo non e' pero' stabile per tutto l'episodio e
termina nella terza stance a `17.676871 s`.

Il probe ha inoltre evidenziato un mismatch interno alla reward. Dopo il TO
precoce dello step `159`, il FSM rifiuta l'evento e resta in
`STANCE_AFTER_HS`, mentre `stance_expected/swing_expected` continua a seguire
lo stato grezzo di `online_gait` e classifica gli step `160-276` come swing.
Il carico della stance viene quindi penalizzato come `swing_unloading` fino al
TO valido dello step `277`.

La sorgente di fase e' stata corretta dando priorita' agli stati FSM accettati
`STANCE_AFTER_HS` e `SWING_AFTER_TO`; il detector grezzo resta fallback durante
`WAIT_HS` o in assenza del payload FSM. Il rerun full-episode mantiene
esattamente dinamica ed eventi (`373` step, `2` cicli, terminazione nella terza
stance), ma rimuove il falso tratto swing:

```text
FSM/reward phase conflicts       118 -> 0
swing_unloading_loss mean      5.882 -> 0.028
swing_unloading_loss max      25.000 -> 2.683
episode_return                -2.743 -> +42.258
discounted return gamma=0.99          = +4.888
```

Il gate reward/FSM e' quindi PASS. Resta separatamente il limite dinamico:
l'oracolo termina per penetrazione nella terza stance e non completa tutti i
`5.0 s`.

### Controfattuale con guard a 30 mm

E' stato ripetuto lo stesso probe mantenendo la soglia soft a `15 mm` e
alzando il guard terminale da `22` a `30 mm`:

```text
steps = 500 / 500
valid HS / TO / cycle = 3 / 3 / 2
invalid events = 2
max penetration = 22.9497 mm
steps above 22 mm = 8
steps above 25 mm = 0
terminated = false
end_reason = episode_time_limit
episode_return = +62.558342
discounted return gamma=0.99 = +5.997279
```

La traiettoria supera quindi il transitorio della terza stance, scarica il
contatto e raggiunge la fine. Il guard `22 mm` era inferiore di circa `0.95 mm`
al picco necessario; `30 mm` e' invece molto piu' ampio del necessario.

Il test non modifica soltanto il momento della terminazione: la scala interna
della loss di penetrazione e' `hard - soft`, quindi passa da `7` a `15 mm` e la
loss massima scende da `1.043` a `0.281`. Prima del training va quindi scelto e
verificato un guard piu' stretto, indicativamente `24-25 mm`, invece di
promuovere direttamente `30 mm`.

### Validazione del guard a 25 mm

E' stato quindi ripetuto lo stesso probe con soglia soft invariata a `15 mm`
e guard terminale a `25 mm`:

```text
steps = 500 / 500
valid HS / TO / cycle = 3 / 3 / 2
invalid events = 2
max penetration = 22.9497 mm
steps above 22 mm = 8
steps above 25 mm = 0
margin below guard = 2.0503 mm
terminated = false
end_reason = episode_time_limit
episode_return = +61.263061
discounted return gamma=0.99 = +5.710611
```

La dinamica e gli eventi FSM coincidono esattamente con il controfattuale a
`30 mm`: nel confronto riga per riga cambiano soltanto `reward` e
`reward_without_grf_slip`. La differenza di return, `-1.295281`, e' quindi
interamente dovuta alla scala piu' severa della loss `hard-soft`, che passa da
`15` a `10 mm`:

```text
penetration loss mean: 0.041449 -> 0.093260
penetration loss max : 0.280880 -> 0.631979
```

Il guard a `25 mm` supera il gate full-episode, conserva un margine di circa
`2.05 mm` sul picco dinamico e penalizza piu' chiaramente l'avvicinamento al
limite rispetto a `30 mm`. E' pertanto il candidato validato per il successivo
confronto fresh-policy versus warm start. Dopo la decisione esplicita, il
valore e' stato fissato nello YAML di training e protetto dagli smoke check di
configurazione; soglia soft, peso e resto della reward restano invariati.

## H1 - Training fresh-policy 10 iterazioni

Run:

```text
Trajectory Generator/runs/training/2026-07-10_H1_contact_support_ledger/
```

Esito tecnico:

```text
ok = true
iterations = 10/10
steps = 40960
elapsed = 4660.65 s
best iteration = 3
best return = -2.104417
```

Trend:

```text
iter 1  return = -2.107897  len = 36.09
iter 3  return = -2.104417  len = 36.03  <- best return
iter 7  return = -2.113677  len = 37.31
iter 10 return = -2.115870  len = 37.45
```

Tutti i `1108` episodi contabilizzati terminano per `grf_penetration`. La
lunghezza cresce di circa `1.37` step, ma resta molto lontana dai circa `120`
step necessari al TO osservato in H0.

## H2 - Rollout best H1

Run:

```text
Trajectory Generator/runs/rollout/2026-07-10_H2_contact_support_ledger_best_recorded/
```

Risultato:

```text
steps = 36
episode_return = -2.121034
reward_mean = -0.058918
terminated = true
valid HS / TO / cycle = 1 / 0 / 0
max penetration = 22.7190 mm
```

Il credito denso provvisorio raggiunge `1.583781`, ma al terminale viene
recuperato integralmente:

```text
contact_support_pending_dense_reward -> 0
contact_support_clawback_penalty      = 1.583781
contact_support_dense_confirmed       = 0
contact_support_to_score              = 0
```

H2 e' quindi FAIL comportamentale, ma il vecchio loophole e' chiuso: la
compressione senza TO non produce piu' un ritorno positivo.

## Decisione

La reward ha superato il gate causale contro la compressione, il gate oracle
online `valid_cycle_count >= 1` e il gate di coerenza fra FSM e termini
stance/swing. Shaping e soglie sono ora congelati nella baseline ex-novo a
`15/25 mm`; il prossimo confronto puo' quindi isolare l'effetto
fresh-policy versus warm-start. Lo stato fisico iniziale degli episodi resta
deterministico in entrambi i casi.

Subito dopo questa correzione resta prioritario validare scientificamente il
warm start actor-only gia' implementato, confrontando a parita' di seed e
configurazione:

```text
fresh-policy init vs warm start imitativo
```

Non e' ancora giustificato introdurre memoria: l'attore osserva gia' stato FSM
e tempo di stance normalizzato, mentre il fallimento avviene prima che serva
una dipendenza temporale lunga.

## File modificati

```text
Trajectory Generator/baseline_MLP/reward_function.py
Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml
Trajectory Generator/baseline_MLP/training_config.py
Trajectory Generator/baseline_MLP/train_ppo_mlp.py
Trajectory Generator/baseline_MLP/rollout_eval.py
Trajectory Generator/baseline_MLP/README.md
validation/prescribed_reward_probe.py
validation/random_policy_reward_probe.py
validation/reward_audit_suite.py
validation/test_reward_function.py
validation/validate_training_config.py
```

## Test e verifiche

```text
reward unit tests                         PASS, 32/32
training config smoke checks             PASS
py_compile file coinvolti                PASS
synthetic negative audit                 PASS, 9/9
prescribed positive audit                PASS
H0 exact-start online-contact            PASS
H0 online-contact HS -> TO -> HS          PASS, 1 ciclo
H0 full episode valid_cycle_count >= 1    PASS, 2 cicli
H0 full episode survival 5 s              FAIL, penetration a 3.73 s
H0 FSM/reward phase consistency           PASS, 0 conflitti
H0 full episode hard guard 30 mm           PASS, 500/500 step
H0 full episode hard guard 25 mm           PASS, 500/500 step
H1 training 10 iterazioni                PASS tecnico
H2 best checkpoint                       FAIL comportamentale
```

## TODO

- [x] Allineare `stance_expected/swing_expected` allo stato FSM accettato dopo
      eventi grezzi rifiutati, evitando la falsa swing penalty.
- [x] Ripetere il probe `training_like` full-episode: due cicli validi e zero
      divergenze fra FSM e reward phase.
- [x] Calibrare un guard di penetrazione `24-25 mm`, mantenendo esplicito
      l'effetto sulla scala della loss soft, e ripetere il solo probe.
- [x] Fissare nello YAML la baseline reward/soglie congelata `15/25 mm`.
- [ ] Eseguire H1 warm-start da 10 iterazioni con reward e soglie congelate.
- [ ] Eseguire H2 deterministico su `rl_module_initial_warm_start` e sul best
      warm-start.
- [ ] Confrontare fresh-policy vs warm-start su return, lunghezza, HS/TO/cicli,
      penetrazione, clawback, clipping e diagnostica SEA.
- [ ] Sbloccare training piu' lunghi solo se H2 warm-start raggiunge almeno un
      TO sinistro valido senza terminazione per penetrazione.
- [ ] Verificare la stabilita' multi-ciclo dell'oracolo online: il TO precoce
      della seconda stance deve essere spiegato o eliminato prima di usare H0
      come riferimento di camminata prolungata.
- [ ] Tenere il meccanismo di memoria in coda finche' il warm start non viene
      valutato e non emerge un limite osservazionale/sequenziale concreto.
