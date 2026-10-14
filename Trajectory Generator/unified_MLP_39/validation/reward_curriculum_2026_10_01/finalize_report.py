"""Write the final report only after the experiment and artifact audit finish."""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
results = json.loads((HERE / 'results.json').read_text())
verification = json.loads((HERE / 'verification.json').read_text())
assert results.get('finished') and verification['passed']
assert results['stop_reason'] == 'stopped_at_quality_gate'
assert results['actor_updates_per_arm'] == 10
assert len(results['evaluations']) == 12
assert all(row['metrics']['steps'] == 500 and row['metrics']['fsm']['end_reason'] == 'episode_time_limit' for row in results['evaluations'])
lookup = {row['label']: row['metrics'] for row in results['evaluations']}
labels = [('CP90 visibile', 'initial_ex_novo_visible'),
          ('CP90 oscurato, nessun update actor', 'initial_ex_novo_hidden'),
          ('Imitazione, 10 update, oscurato', 'imitation_s1_u10_hidden'),
          ('Ex novo, 10 update, oscurato', 'ex_novo_s1_u10_hidden'),
          ('Ex novo, 10 update, visibile', 'ex_novo_s1_u10_visible')]
table = ['| Policy | ROM knee/ankle [°] | RMSE actual–prescribed [°] | RMSE actual–served [°] | Cicli validi |',
         '|---|---:|---:|---:|---:|']
for title, label in labels:
    m = lookup[label]
    joints = [m['joints'][j] for j in ('knee', 'ankle')]
    pair = lambda key: '/'.join(f'{j[key]:.2f}' for j in joints)
    rom = '/'.join(f'{j["actual"]["excursion_deg"]:.2f}' for j in joints)
    table.append(f'| {title} | {rom} | {pair("rmse_actual_target_deg")} | {pair("rmse_actual_served_deg")} | {m["fsm"]["phase_valid_cycle_count"]} |')
torque = ['| Policy oscurata | Ankle area positiva [N m s] | Knee spring HP20 RMS [N m] |',
          '|---|---:|---:|']
for title, label in labels[1:4]:
    m = lookup[label]
    torque.append(f'| {title} | {m["torque"]["ankle"]["positive_area_nm_s"]:.3f} | {m["torque"]["knee"]["highpass_rms_nm"]["20"]:.3f} |')
elapsed = (results['finished'] - results['started']) / 3600
text = f'''# Reward ex novo durante oscuramento progressivo — 2026-10-01

**Implementazione e confronto completati. La sostituzione della reward non ha recuperato il passo oscurato entro il budget di questo test.** Entrambi i rami hanno eseguito un warm-up del critic e dieci aggiornamenti dell'actor da CP90. Dopo il secondo controllo il curriculum si è fermato al primo livello: 75% degli episodi visibili, 25% oscurati. I livelli successivi 50%, 25%, 0% di visibilità **non sono stati eseguiti**, perché il rollout ex novo completamente oscurato non supera le soglie preregistrate. Non è una policy promossa per funzionamento autonomo.

## Risultato fisico

Tutti i 12 rollout completano 500 passi, con termine normale. Finestra comune 13.19–17.94 s; cinematica fisica, non sola uscita della rete. Il criterio richiede almeno due cicli validi, ROM knee ≥20° e ankle ≥15°, tracking actual–served ≤2° e completamento normale. Entrambi i rami oscurati falliscono ROM e numero di cicli dopo cinque e dopo dieci aggiornamenti. Il criterio non richiede di coincidere temporalmente con la prescribed: il fallimento di ampiezza non si spiega con il solo sfasamento.

{chr(10).join(table)}

Il tracking actual–served rimane sotto 0.3° nei rollout oscurati: il limite osservato è nella traiettoria generata, non nell'inseguimento locale dei SEA. Il ramo ex novo visibile conserva un movimento ampio; in questa prova non emerge una perdita generale della capacità quando gli ingressi prescribed sono presenti. La perdita immediata oscurando CP90 precede qualsiasi aggiornamento dell'actor.

{chr(10).join(torque)}

La riduzione dell'area positiva ankle non dimostra recupero cinematico. Nel controllo imitativo dopo cinque aggiornamenti l'area scende 2.43→1.35 N m s mentre il ROM ankle peggiora 10.26→8.41°. Dopo dieci aggiornamenti entrambi restano sotto soglia; il chattering knee misurato sopra 20 Hz cambia poco. I return numerici non sono confrontati tra reward diverse.

- [Tutte le metriche cinematiche](metrics_table.md), [coppie](torque_table.md), [confronti sulla stessa finestra](paired_windows.json).
- [Curve cinematiche oscurate, 10 update](../../../../plot/10_01_2026_reward_curriculum_comparison/s1_u10_hidden_kinematics.png).
- [Curve cinematiche visibili, 10 update](../../../../plot/10_01_2026_reward_curriculum_comparison/s1_u10_visible_kinematics.png).
- [Coppie oscurate, 10 update](../../../../plot/10_01_2026_reward_curriculum_comparison/s1_u10_hidden_torques.png).
- [Corridoio morphology ex novo oscurato](../../../../plot/10_01_2026_reward_curriculum_comparison/ex_novo_s1_u10_hidden_morphology.png).
- [Indice e hash degli 84 plot canonici](plots_verified.json), sette per rollout nelle sottocartelle del root `plot/`.

## Cosa significa per l'ipotesi sulla reward

Usare la stessa reward ex novo durante tutto l'oscuramento è ora tecnicamente possibile ed è stato provato mantenendone formula e pesi attivi. **In questa ricetta di trasferimento, cambiare obiettivo da solo non basta.** Non emerge un vantaggio cinematico netto dell'ex novo sul controllo imitativo: a dieci aggiornamenti il controllo ha più ROM knee e minore RMSE ankle, ma nessuno recupera un passo sufficiente.

Questo non è un test definitivo contro la reward ex novo. Sono presenti tre limiti misurati:

1. **Adattamento dell'actor molto piccolo.** LR 5e−7, una epoca e sigma 0.01: sugli stessi 40 stati di prova, le medie degli endpoint ex novo cambiano di circa 0.019°/0.014° RMS nei 20 stati oscurati; imitazione 0.049°/0.012°. Sono sonde locali prima del clipping, non una previsione delle curve a ciclo chiuso. Il regime ereditato da J20 è conservativo e nasce per rifinire una policy già competente senza teacher; CP90 perde ampiezza non appena gli ingressi vengono rimossi.
2. **Critic non calibrato dopo il warm-up.** Ultima explained variance −0.0034 ex novo e −0.2068 imitazione. La loss VF libera/clippata è 338.07/6.39 e 25.99/9.43. Il codice locale RLlib satura `(V−target)²` a 10: sui campioni con errore assoluto >√10 il termine VF non produce gradiente. La frazione esatta di campioni saturati non è salvata. Il warm-up ha aggiornato i pesi senza cambiare l'actor, ma questo non prova l'adattamento del valore al nuovo obiettivo. Le tracce di rollout non contengono tutto lo stato privilegiato del critic, quindi non è stata inventata una calibrazione offline da osservazioni incomplete.
3. **Obiettivo e copertura morphology.** Già prima del training, l'ex novo assegna −342.42 al rollout oscurato compresso e −680.32 a quello visibile: costo morphology 133.87 contro 682.26, con 258 contro 484 step disponibili. Dopo dieci aggiornamenti il rollout ex novo oscurato ha 252 campioni risolti e 248 scartati, mentre quello visibile ne risolve 488. Differiscono anche fase, forma e penalità temporali: la sola copertura non spiega tutta la differenza. Questo impedisce di usare il return come certificazione della cinematica. Il profilo morphology e il target imitativo hanno preprocessing e allineamento dei cicli differenti; chiedono riferimenti non necessariamente coincidenti.

La via resta plausibile, ma il prossimo test informativo dovrebbe prima verificare una calibrazione effettiva del critic e un adattamento dell'actor sufficiente agli ingressi oscurati, mantenendo poi la stessa reward ex novo. Il precedente successo ex novo con un parent già adattato senza teacher non equivale al trasferimento diretto da questo CP90. Nessuna di queste ulteriori prove è dichiarata eseguita qui.

Vedi [diagnosi iniziale dettagliata](initial_reward_diagnosis.md), [telemetria per aggiornamento](training_health_summary.md), [aggregati](training_aggregate.json) e [sonda delle medie](fixed_probe_drift.json).

## Disegno del confronto

Stesso actor CP90 del run 29 settembre, critic e Adam nuovi con seed 123, un warm-up critic-only, sigma 0.01 congelata, stessa fisica/governor/reset/start 1 s, episodi 5 s, batch 4096, 13 worker, una epoca, LR 5e−7, PPO clip 0.05 e guard KL 0.01. Sono 45.056 passi per ramo: 4.096 warm-up e 40.960 actor. Quota visibile effettiva durante gli update actor 77.91% ex novo e 77.72% imitazione. I seed iniziali e il calendario coincidono; le singole maschere successive possono divergere per le diverse durate degli episodi.

La reward ex novo coincide esattamente col preset J20 su 500 passi storici. Il controllo usa i coefficienti imitativi CP90, con i costi comuni e i criteri FSM del runtime J20 applicati a entrambi; non è una ripetizione del vecchio pilot CP90. Il dizionario di configurazione differisce soltanto per `reward_mode` e cartella di output. Slew e fisica restano CP90, quindi questo non è una riproduzione completa di ogni impostazione fisica J20. La penalità morphology è inattiva nello scalare imitativo, ma le terminazioni del suo contratto sono allineate. Nessun cambiamento al simulatore root o al plugin SEA.

Il checkpoint viene valutato dopo 5 e 10 update; non viene scelto retroattivamente in base al return. Il curriculum avanza in funzione del rollout ex novo oscurato; il controllo segue lo stesso calendario. Un solo seed, un solo start: screening condizionato al protocollo, non prova di robustezza. Tempo del confronto principale, comprendente rollout iniziali e ripresa dell'orchestrazione: circa {elapsed:.2f} ore; sviluppo e smoke sono esterni a questa misura.

[Protocollo preregistrato](PROTOCOL.md), [config ex novo](ex_novo.yaml), [config imitazione](imitation.yaml), [differenze](configuration_difference.json), [parità reward](reward_parity.json), [risultati completi](results.json).

## Modifiche software

- `target_mask_mode: episode` separa disponibilità prescribed e reward, in ambiente e modulo della rete. La maschera è estratta per episodio e conservata nel batch; nessuna nuova estrazione durante PPO. Il default `stage_default` mantiene il comportamento storico ex novo sempre oscurato.
- `actor_transfer_warmup` permette trasferimento dell'actor, critic/optimizer nuovi e warm-up sotto entrambi gli obiettivi; sigma fissa e congelata anche nel ramo imitativo. Stato, export e resume conservano i nuovi contratti.
- `morphology_termination_mode: ex_novo` allinea esplicitamente le terminazioni fisiche del confronto senza aggiungere la penalità morphology allo scalare imitativo.
- Learner di audit KL per start singolo: riusa la telemetria per minibatch esistente senza il connettore per il campionamento esatto a tre start.

File: [target_availability.py](../../target_availability.py), [env.py](../../env.py), [env_factory.py](../../env_factory.py), [asymmetric_rl_module.py](../../asymmetric_rl_module.py), [training_config.py](../../training_config.py), [training_runtime.py](../../training_runtime.py), [train_ppo_mlp.py](../../train_ppo_mlp.py), [reward_function.py](../../reward_function.py), [reward_resume.py](../../reward_resume.py), [rollout_eval.py](../../rollout_eval.py), nuovo [kl_audit_learner.py](../../kl_audit_learner.py), [test_reward_curriculum.py](../../tests/test_reward_curriculum.py) e [README del modulo](../../README.md). Harness e analisi sono nella presente directory. I file originari coinvolti sono conservati in `before/`; nessun reset del worktree preesistente.

## Verifiche e incidenti conservati

**247 test passati**, smoke nativo con warm-up e update actor per entrambi gli obiettivi, smoke nativo di resume/cambio curriculum, parità fisica iniziale esatta fra obiettivi a pari visibilità, reward parity, sigma costante in tutti i checkpoint, continuità actor/critic fra aggiornamenti, ripristino Adam e coefficiente KL, guard KL superato in tutte le 22 iterazioni principali. Massimo KL per minibatch 0.001596 <0.01. Verificati 12 rollout e 84 PNG canonici; hash di CP90 e codice eseguito invariati.

La prima prova sandbox è stata fermata da Ray prima del training; un primo smoke nativo è stato fermato dopo il solo critic per assenza di telemetria KL. Entrambi sono archiviati. Il controllo dell'orchestrazione dopo il resume ex novo confrontava erroneamente iterazioni cumulative e aggiunte: il training era completato e il checkpoint valido. La continuazione separata usa `iterations_completed_this_process`, riutilizza il training concluso e non ripete update; codice di training invariato. [Dettaglio della ripresa](orchestration_recovery.md).

[Verifica finale](verification.json), [test](test_verification.json), [smoke](smoke_verification.json), [resume nativo](native_resume_verification.json), [parità fisica](initial_physical_parity.json), [manifest originale](execution_code_manifest.json), [manifest della continuazione](continuation_code_manifest.json).
'''
(HERE / 'README.md').write_text(text)
print('Final report written:', HERE / 'README.md')
