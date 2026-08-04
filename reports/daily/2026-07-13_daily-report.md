# Daily Report - 2026-07-13

Instruction check token: CMC_AGENT_OK_2026

## Report utente consolidati

- `reports/user/2026-07-13_actor_robust_deployable_critic_warmup.md`
- `reports/user/2026-07-13_ablation_memoria_controller_actor.md`
- `reports/user/2026-07-13_sblocco_actor_markov35_e_warmup_critic.md`
- `reports/user/2026-07-13_h1_markov35_post_warmup_critic_h2_respinta.md`
- `reports/user/2026-07-13_h1bis_multistart_optimizer_resume_h2_approvata.md`

## Sintesi

La giornata ha risolto due blocchi distinti: la fragilita' dell'actor a
`sigma=0.005` e l'instabilita' del primo update PPO dopo il warm-up del critic.

Una prima diagnosi ha mostrato che lo stato interno del controller poteva
diventare un proxy autoregressivo fragile. Il contratto a 25 feature ha
dimostrato robustezza, ma la successiva verifica storica ha chiarito che le
dieci feature di stato controller sono deployable e rendono il processo
Markoviano. Il primo candidato a 35 feature era fallito per un dataset di
recovery errato, non per l'architettura.

Dopo aver riallineato le label alla fase FSM e scalato gli ingressi, il nuovo
actor 35-feature ha superato tre start deterministici e tre seed stocastici. Il
critic e' stato riscaldato con actor congelato. Una prima H1 standard e' stata
respinta; una H1-bis con sampling multi-start e LR reale corretta dopo il
restore optimizer ha invece superato H2 completa.

## 1. Diagnosi della fragilita' e baseline 25-feature

Un trace corretto a `sigma=0.005` ha mostrato che la media actor iniziava a
derivare prima del mismatch FSM. Sostituire lo stato nominale del
controller/reference eliminava il `47.52%` della deriva. Previous endpoint,
reference servita e ultimo comando SEA erano stati usati durante l'imitazione
come proxy della prossima azione.

Come correzione strutturale iniziale, queste 14 feature sono state spostate nel
suffisso privilegiato del critic. L'actor a 25 feature, senza prescribed,
sound-side state o stato biologico, ha ottenuto:

```text
deterministico                    500 step, 2 cicli
sigma=0.005 seed 123/124/125      500 step, 3/3 PASS
clipping                          0%
```

Il critic-only warm-up ha raggiunto explained variance `0.598370`, mantenendo
l'actor bit-identico. Questo risultato ha dimostrato che la fragilita' a
`sigma=0.003` non era imposta da reward o dinamica.

## 2. Ablation e rettifica sullo stato Markov

I report storici indicavano pero' che endpoint precedente, reference servita e
comando SEA sono stato interno disponibile anche in deployment. Il primo
tentativo di reintrodurli ha espanso l'actor da 25 a 35 feature, addestrando
solo le nuove colonne del primo layer.

Il candidato conservativo passava il nominale ma falliva lo start `+0.20 s` a
52 step. Fu correttamente respinto dal gate, ma il risultato non dimostrava che
lo stato Markov fosse dannoso.

Una diagnosi piu' profonda ha trovato la causa reale nel dataset DAgger:

```text
campioni recovery                         1500
campioni ancora allineati alla FSM         356
campioni con label fuori fase             1144, 76.27%
primo mismatch nei trace                  step 119-120
```

Le azioni teacher erano etichettate per indice temporale anche dopo che la
traiettoria perturbata aveva cambiato fase discreta. Inoltre le accelerazioni
del reference arrivavano a `60 rad/s^2` e dominavano numericamente gli input.

La correzione ha introdotto:

- troncamento delle trace al primo mismatch discreto;
- scaling fisico di velocita' e accelerazioni, assorbito poi nel primo layer;
- pesi indipendenti per gli start alternativi;
- adattamento controllato dell'intera mean-network;
- `log-std` congelato e critic escluso dall'adattamento.

## 3. Actor Markov35 selezionato

Il dataset phase-aligned finale conteneva 24.712 campioni tra ancore nominali,
recovery stocastici validi e teacher sugli start `-0.20/+0.20 s`.

Checkpoint selezionato:

```text
Trajectory Generator/runs/training/
target_domain_markov35_phase_aligned_scaled_full_r32_alt8_2026-07-13/
rl_module_target_adapted
actor digest: a0801a9e635db4f2973da7d8f6461cbbf7b1643efef1dedc2baafd9c9f95ca21
```

Validazione deterministica:

| Start | Step | Return | Cicli | Max penetrazione |
| --- | ---: | ---: | ---: | ---: |
| nominale | 500 | 51.797 | 2 | 23.960 mm |
| `-0.20 s` | 500 | 35.788 | 2 | 24.602 mm |
| `+0.20 s` | 500 | 55.926 | 2 | 24.324 mm |

Validazione stocastica nominale a `sigma=0.005`: seed 123/124/125 tutti a 500
step, tre cicli e clipping nullo. Un refinement con loss offline migliore e'
stato respinto perche' lo start `-0.20 s` terminava a 221 step: il gate
closed-loop ha avuto priorita' sulla metrica supervisionata.

Contratto finale selezionato:

```text
actor                               35 feature Markov deployable
controller state actor-visible      si
diagnostiche ridondanti              no, critic-only
prescribed actor-visible             no
critic privilegiato                  si, 84 feature
gait clock                           disabilitato
sigma                                0.005
soglie                               15/25 mm
```

## 4. Warm-up del critic

Il critic e' stato addestrato per una sola iterazione su 4096 step con actor
completamente congelato:

```text
vf explained variance           0.238068
mean KL                        0.0
actor max change               0.0
critic restore                 digest identico
```

Checkpoint completo canonico per il resume:

```text
validation/critic_warmup/
2026-07-13_markov35_phase_aligned_sigma0005_iter1_retry/checkpoint_last
```

E' stato anche corretto un errore di sola provenance nel parser della config
adiacente al warm-start source. Pesi e comportamento del warm-up valido non
sono cambiati.

## 5. Prima H1 post warm-up respinta

La prima H1 con 10 epoch, LR `1e-4`, `sigma=0.005` e `log-std` congelato ha
prodotto KL `0.154690`, circa 15.5 volte il target. H2 deterministico ha
conservato nominale e `+0.20 s`, ma lo start `-0.20 s` e' terminato a 231 step
per penetrazione `25.334 mm`.

Il checkpoint e' stato respinto. Reward, guard e actor Markov non sono stati
modificati: il failure riguardava lo specifico update PPO, non il warm start.

## 6. H1-bis multi-start e correzione optimizer

Sono emersi due difetti di orchestrazione:

1. `worker_index` veniva perso quando `EnvContext` era convertito in `dict`,
   quindi il batch non distribuiva realmente gli start;
2. il restore del checkpoint ripristinava anche la LR `1e-4` nei param group di
   Adam, annullando le LR piu' piccole dichiarate nella nuova configurazione.

L'assegnazione dello start e' stata spostata nel callback
`on_environment_created`; il trainer riapplica ora la LR risolta subito dopo
il restore e ne registra l'audit.

Copertura batch valida:

```text
-0.20 s                    1576 step
nominale                   1260 step
+0.20 s                    1260 step
```

Configurazione H1-bis valida:

```text
una iterazione / una epoch
LR Adam reale                      5e-7
clip_param                         0.05
kl_coeff / kl_target               1.0 / 0.01
sigma                              0.005
freeze_logstd                      true
```

Risultato:

```text
KL RLlib                           0.0005523
KL empirico                        0.000605-0.000668
RMSE spostamento media             0.000123-0.000129
max delta parametro actor          3.892e-6
log-std                            invariato esattamente
```

H2 completa ha passato tutti i sei rollout, tre start deterministici e tre seed
stocastici. Ogni episodio ha completato `500/500` step, almeno due cicli,
penetrazione inferiore a 25 mm e clipping nullo.

Stato finale:

```text
actor Markov35 warm start           PROMOSSO come H0
critic warm-up                      PASS
prima H1 a 1e-4                     REJECT_H1
H1-bis a LR reale 5e-7              PASS
H2 deterministico/stocastico        6/6 PASS
training lungo                      non ancora autorizzato
```

## File modificati

- `Trajectory Generator/osim_trj_cmc_like.py`
- `Trajectory Generator/baseline_MLP/asymmetric_rl_module.py`
- `Trajectory Generator/baseline_MLP/training_config.py`
- `Trajectory Generator/baseline_MLP/env_factory.py`
- `Trajectory Generator/baseline_MLP/tb_logging.py`
- `Trajectory Generator/baseline_MLP/train_ppo_mlp.py`
- `Trajectory Generator/baseline_MLP/rollout_eval.py`
- `Trajectory Generator/baseline_MLP/target_domain_imitation.py`
- `Trajectory Generator/baseline_MLP/target_domain_markov_adaptation.py`
- `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml`
- `validation/diagnose_actor_fragility.py`
- `validation/diagnose_markov_state_adaptation.py`
- `validation/validate_training_config.py`
- `validation/test_env_start_assignment.py`
- `validation/test_training_resume.py`
- test focalizzati per RLModule, porting, adaptation e configurazione.

## Test e verifiche

- suite unitaria finale: `63/63` PASS;
- config smoke train/rollout: PASS;
- `py_compile`: PASS;
- `git diff --check`: PASS;
- actor adaptation save/reload: bit-exact;
- actor dopo critic warm-up: bit-exact;
- restore critic full checkpoint: digest identico;
- audit LR optimizer reale dopo restore: PASS;
- H2 finale seriale: `6/6` PASS;
- nessun processo Ray/OpenSim residuo.

## TODO aperti e propagati

- [x] Eseguire critic warm-up e una nuova H1 con budget KL controllato. Il
      warm-up e H1-bis sono stati completati; H2 finale ha passato `6/6`
      rollout.
- [x] Chiudere il confronto H1 lungo contro fresh-policy: il protocollo a 10
      iterazioni e' stato sostituito dai gate per singolo update con rollback.
- [ ] Continuare con un blocco PPO breve e controllato, ripartendo dal
      checkpoint H0 completo e provando gradualmente LR `1e-6`.
- [ ] Mantenere sampling multi-start verificabile, `sigma=0.005`,
      `freeze_logstd=true`, audit della LR reale e gate KL `<= 0.01`.
- [ ] Ripetere probe critici e H2 completa prima di promuovere ogni checkpoint;
      non avviare ancora 100 iterazioni.
- [ ] Aggiungere ai gate futuri una misura esplicita del picco reserve, non
      riportata nella H2 finale nonostante il full episode `6/6`.
- [ ] Estendere la validazione a trial, velocita' e soggetti non usati
      nell'adattamento prima di dichiarare deployment validato.
- [ ] Il TODO memoria e' parzialmente risolto: lo stato controller Markov e'
      stato reintrodotto e validato senza prescribed. Una memoria ricorrente
      resta differita finche' non emerga un limite sequenziale misurabile.
- [ ] Spiegare il TO precoce rifiutato nella seconda stance dell'oracolo
      multi-ciclo, TODO storico ancora non chiuso.

## TODO storico SEA propagato

- [ ] Valutare una deflessione SEA iniziale coerente con la coppia richiesta;
      il punto progettuale del 13/06 non risulta ancora formalmente chiuso.
