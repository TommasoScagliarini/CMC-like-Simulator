# Audit indipendente: reset IK della policy V26C ex-novo

Ho usato solo Read/Grep/Glob, senza simulazioni né servizi esterni. Non ho aperto audit o report del 12/10; la cartella `reset_ik_audit_2026_10_12` non esiste nel repo.

Percorsi:
- `osim_trj_cmc_like.py`, `prosthetic_phase_fsm.py` e `binary_phase_adapter*.py` stanno in `Trajectory Generator/`.
- `env_factory.py`, `train_ppo_mlp.py`, `reward_function.py`, `asymmetric_rl_module.py`, `tb_logging.py`, `start_sampling.py` e `start_condition_metrics.py` stanno in `Trajectory Generator/baseline_MLP/`.
- **YAML** è il `training_cfg.resolved.yaml` del run del 01/09. **RO** è `runs/rollout/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-02_best_rollout/`.

**Esito sull'ipotesi**
- **Vero:** a ogni reset q e qdot di tutte le 21 coordinate vengono dall'IK, senza rumore.
- **Falso come formulato:** l'IK al reset non è l'unica informazione prescribed.
  - Il critic riceve l'IK biologica a ogni passo.
  - Il plant insegue l'IK su 19 DOF e applica la GRF destra presa da file.
  - Il reward usa un corridoio morfologico derivato dallo stesso file IK.
  - Solo l'input esplicito dell'actor è privo di IK dopo t0.
- **Non dimostrato:** che la ripetizione della posa limiti l'apprendimento.

**Limite di provenienza:** le righe citate sono del working tree attuale. Secondo git status ci sono modifiche non committate, tra cui `osim_trj_cmc_like.py` e `train_ppo_mlp.py`, e `summary.json` non registra hash del codice. Il percorso di reset è però confermato numericamente dagli artefatti del 02/09.

## 1. Percorso al reset: q/qdot, SEA, governor, FSM

La catena è `reset()` → `_initialise_episode()` (`osim_trj_cmc_like.py:1786-1799`, `:2536-2611`).

- **Start:** con `random_init: false` (YAML:40), t0 = `t_start` (11.99, `AB06_SEASEA_stiff321_500_pi_setup.xml:8`) + offset. L'offset è limitato a `t_end − episode_duration` (`osim_trj_cmc_like.py:2542-2553`).
- **Riferimento:** `kin.clear_segment()` (:2565 → :532-539) cancella la spline e lo stato del governor. Con `imitation_initialize_to_target: false` (YAML:68) non c'è segmento, quindi `kin.get(t)` restituisce l'IK base (:829-830).
- **q e qdot:** `reset_to_time` (`simulation_runner.py:335-400`) prende valore e derivata prima della spline IK, filtrata a 6 Hz (`kinematics_interpolator.py:184-187`, `config.py:142-145`).
  - `_set_state` le scrive per ogni coordinata (`simulation_runner.py:346-347`, `:2663-2673`).
  - Le 21 colonne IK (`IK_results_AB06_SEASEA.mot:11`) coincidono con le 21 coordinate dell'`.osim`, quindi nessuno stato resta dall'episodio precedente.
  - I giunti protesici partono dalle colonne IK `pros_knee_angle` e `pros_ankle_angle` di AB06.
- **Muscoli:** fiber_length ottimale e activation 0.01, valori non derivati dall'IK (`simulation_runner.py:1762-1776`). Il SO li ri-equilibra a ogni finestra da 1 ms (:873-875).
- **SEA:**
  - Le variabili di stato del plugin vengono azzerate (:1778-1794).
  - Poi motor_angle = q_IK e motor_speed = qdot_IK (:1815-1842): la molla non è deflessa e τ_spring = 0 a t0.
  - Controlli a 0 (:360), integratori e LPF del controllore cascade azzerati (`prosthesis_controller.py:284-292`), warm start del SO (:375-379).
- **Governor:** si inizializza al primo `step`, non al reset.
  - L'ancora è `kin.get(t0)`, cioè q e qdot IK (`osim_trj_cmc_like.py:2683-2704`).
  - Il modello Butterworth3 jerk-limited a 4 Hz (YAML:53-55) parte da `_filt_q = q_IK`, `_filt_v = clip(qdot_IK)`, `_filt_a = 0` (:707-728).
  - Anche lo slew limiter è ancorato a q_IK (:2596-2600, :2691-2696). Al primo passo il target si sposta al massimo di 0.025 rad al ginocchio e 0.02 rad alla caviglia (YAML:51-52).
- **FSM:** dopo `reset()` (:2609) viene inizializzata dal campione binario heel/toe nella posa IK (`simulation_runner.py:388-399`, `osim_trj_cmc_like.py:3179-3268`, `binary_phase_adapter.py:91-101`, `prosthetic_phase_fsm.py:243-279`).
  - Piede in aria → WAIT_HS; piede in contatto → stance parziale non accreditata.
  - Lo stato iniziale è quindi una funzione deterministica della geometria della posa IK.

**Conferma dagli artefatti:** a start 1.957 s (t = 13.94687), `RO/rollout_reset_diagnostics.json:12-41` mostra actual = prescribed = served.

| giunto | q | qdot |
|---|---|---|
| ginocchio | −0.1742 | −0.0418 rad/s |
| caviglia | 0.0518 | −0.7165 rad/s |

Angolo e velocità motore coincidono con q e qdot. La velocità non è zero: è uno stato di metà passo.

**Due artefatti di reset nell'input dell'actor**, identici a ogni episodio:
- `served_ref_accel` al reset vale la qddot IK: −14.70 e −15.99 rad/s² (`RO/rollout_policy_trace.json:44,49`). Il governor però parte da a = 0, e dopo il primo passo il valore è −2.46 (:126). È anche l'unico valore non misurabile in deploy (oracolo) nell'osservazione dell'actor.
- Carico e contatto sinistro valgono 0, perché non c'è ancora un campione, mentre la FSM è già in `stance_after_hs=1` (:26-27, :33-37).

## 2. Start configurati e usati

**Configurazione**
- Offset [1.7569, 1.9569, 2.1569] s (YAML:42-45), `exact_start_sampling: true` (:33), 12 runner (:31).
- Un solo env per runner (`train_ppo_mlp.py:1491`).
- Il contratto esatto vieta `random_init` (`start_sampling.py:63-66`) e richiede esattamente 3 offset (`train_ppo_mlp.py:4290-4294`).

**Assegnazione: uno start fisso per runner, non per episodio**
- Formula: `offsets[(worker_index−1+vector_index) % 3]` (`env_factory.py:74-81`), applicata via `on_environment_created` (`tb_logging.py:331-334`, `env_factory.py:84-112`).
- Runner 1,4,7,10 → 1.757 s; 2,5,8,11 → 1.957 s; 3,6,9,12 → 2.157 s. In tempi assoluti: 13.747, 13.947, 14.147 s.
- Tutti gli episodi di un runner, e i 4 runner dello stesso gruppo, partono dallo stesso stato. Nell'intero run ci sono solo 3 stati iniziali.
- In `osim_trj_cmc_like.py` non c'è alcun rumore o perturbazione al reset.

**Uso reale**
- `exact_start_balance.actual_steps` = 1536 per start (`summary.json:1272-1276`); 384 step per runner, 4 runner per start (:898-900).
- In 38 iterazioni su 50 `completed_episodes` vale 4.0 per start: 152 episodi per start, tutti con `episode_length_mean: 500.0`.
- `env_runners/reward_loss/terminated` vale 0.0 in tutte le 50 righe: i reset avvengono solo all'orizzonte di 5 s.
- Le iterazioni senza episodi completati (2, 6, 10, 14, 19, 23, 27, 32, 36, 40, 45, 49) sono le stesse per i tre start. Tutti i runner sono quindi sincronizzati (lockstep) sulla stessa posizione dentro l'episodio.

**Lineage**
- Il run riparte (`resume_from`, `summary.json:22`) dal critic-rewarm r2. Quel run usava un solo start, 1.957 s, con actor congelato e lr 1e-4 (suo YAML:14, 20, 38-40).
- Il pilot del 28/08 usava già i 3 start.
- Tutti i rollout storici AB06 V26C sono a 1.957 s.

| start | return medio (38 gruppi da 4 episodi) | range | iterazioni con `advantage_mean` < 0 |
|---|---|---|---|
| 1.757 s | 50.2 | 36.5–53.2 | 6/50 |
| 1.957 s | 25.7 | 23.4–28.0 | 47/50 |
| 2.157 s | 31.4 | 22.8–35.7 | 40/50 |

- Nessuno start mostra un trend. Il return globale resta tra 32.6 e 38.1 (best 38.06, `summary.json:888`).
- `vf_explained_var` resta tra −0.05 e 0.13.
- Gli advantage per start sono quelli standardizzati sul batch e usati dal learner (`start_condition_metrics.py:831-855`).
- Le tre finestre da 5 s si sovrappongono per 4.6–4.8 s.

## 3. Actor 35 / critic 84

**Split:** l'env emette il vettore [actor | privilegiate] (`train_ppo_mlp.py:1431`). La policy legge `obs[:n_actor]`, il critic il vettore intero (`asymmetric_rl_module.py:131-150`).

**Actor** (`osim_trj_cmc_like.py:2939-3088`, verificato sulla traccia RO :15-50):

| feature | n | valore al reset | dopo t0 |
|---|---|---|---|
| `gait_phase_sin/cos` | 2 | 0 / 1 | costanti, il clock è disabilitato (YAML:62; :2250-2251; :911-912) |
| q, qdot protesici | 4 | IK | stato simulato |
| angolo e velocità motore SEA | 4 | q, qdot IK | stato del plugin |
| carico e contatto sinistro | 2 | 0 | GRF online applicata |
| HS, TO, fase sin/cos, durata ciclo sinistro | 5 | 0, 0, 0, 1, 0 | detector |
| FSM | 8 | dalla posa IK | eventi V26 |
| previous_endpoint, served_ref q/v/a, sea_u (×2 giunti) | 10 | q, q, qdot, qddot IK, 0 | governor + policy |

Al reset 16 feature su 35 sono valori IK. Le altre 19 sono costanti, zeri o funzioni deterministiche della posa IK.

**Critic:** le 35 dell'actor più 49 privilegiate.
- `sea_u_abs/saturated` (4).
- `pros_*_target/_vel/_tracking_error` (6). Il commento a :3091 dice "IK reference", ma il codice usa `q_ref` servito (:2947, :3096-3100). Sono IK solo a t0, poi duplicano `served_ref`.
- 11 coordinate biologiche × (q, vel, `_kin_ref`) (33; :3104-3108, :4006-4024). `_kin_ref` è l'IK(t) a ogni passo: un segnale prescribed diretto e continuo.
- Piede destro (6): un sensore online virtuale su un piede che segue l'IK.

**Segnali prescribed**
- **Diretti nell'actor:** solo nell'osservazione di reset. Dopo t0 restano solo memorie inizializzate dall'IK: slew limiter, governor, stato fisico, FSM fino ai primi eventi validi.
- **Indiretti nell'actor:** tutte le osservazioni fisiche escono da un plant guidato dall'IK. L'anca sinistra (moncone) segue l'IK, quindi il ritmo della coscia e il contatto del piede protesico li impone il dataset.
- **Supporto biologico:**
  - Ogni 1 ms il controllore esterno (PD + feedforward) insegue l'IK su 19 DOF (`outer_loop.py:97-113`, `simulation_runner.py:865-871`, `model_loader.py:1166`).
  - Guadagni Kp/Kd 100/20, 25/10 sulle traslazioni pelviche (`config.py:258-271`).
  - ID + SO con reserve; i residui pelvici hanno un limite di 1000 (`config.py:303-308`).
  - L'equilibrio lo garantiscono i residui, e il reward non li penalizza (YAML:189, 192).
- **GRF:** la destra è prescritta da file. Il prescritto è disattivato solo sul lato sinistro, dove la GRF online è applicata (`model_loader.py:943-945, 1022`; YAML:88-90).
- **Reward:**
  - I termini di tracking, reference e bio sono a 0 (YAML:195-197, `reward_function.py:991-1002`).
  - La morfologia, con peso 5 (YAML:216-218), usa però un corridoio media±std calcolato sulle colonne IK protesiche dello stesso file. Il corridoio è indicizzato dalla FSM (profilo JSON:1033 `ik_source`, :1039 `n_cycles: 123`).
  - Le costanti temporali, per esempio `phase_period_nominal_s` 1.58, sono probabilmente calibrate sui dati; non l'ho verificato.

## 4. Cosa è dimostrato e cosa richiede un esperimento causale

**Dimostrato** (descrittivo, non causale)
- Il reset è coerente con l'IK, completo e deterministico. Ci sono 3 stati iniziali, fissi per runner e sincronizzati.
- L'ipotesi "unica informazione prescribed" è falsa per critic, plant e reward. L'assenza di target IK nell'actor non rende autonomo il plant.
- Il return dipende fortemente dallo start (circa ×2), in modo persistente, anche con finestre sovrapposte al 92–96%.
- Il segno dell'advantage è stabile per start e il critic predice poco.
- Il regime di ottimizzazione è quasi fermo:
  - lr 5e-7, con un solo optimizer per actor e critic (`summary.json:872-886`);
  - σ fissa a 0.01, clip 0.05, 1 epoca;
  - KL medio ≈ 2–3·10⁻⁴ nelle iterazioni che ho controllato.

**Non dimostrato**
- Che la ripetizione dei 3 stati iniziali limiti l'apprendimento.
- Cosa causa il divario per start. I candidati sono: posa e velocità iniziali, finestra prescritta, accredito FSM, transitori di reset, o un critic calibrato su 1.957 s che con lr 5e-7 non riesce a ricalibrarsi.
- Che un reset diverso migliori l'apprendimento.

Un test a policy congelata su start nuovi misura solo la robustezza della policy già addestrata, non un effetto causale sull'apprendimento.

## 5. Test minimi e confondenti

1. **T0 – audit del reset per start, senza training.** Per i 3 start e per una griglia su un ciclo (~1.58 s):
   - registrare diagnostica di reset, contatto heel/toe e stato FSM;
   - eseguire un rollout scriptato "hold" di 0.1–0.3 s registrando residui e reserve, saturazione SEA, penetrazione e carico.
2. **T1 – sweep a policy congelata** sulla stessa griglia, con più seed. Dà una mappa di generalizzazione della policy esistente; non è causale.
3. **T2 – A/B causale.**
   - Braccio A: 3 start per runner, come ora.
   - Braccio B: 12 start equispaziati su un ciclo, reset IK invariato.
   - Tutto il resto identico: stesso punto di partenza con critic pre-calibrato sulla stessa distribuzione multi-start, iperparametri in un regime dove l'apprendimento si vede, almeno 3 seed per braccio, stesso budget, valutazione su start mai visti e comuni ai due bracci.
   - Il contratto attuale accetta solo 3 offset: serve una variante esplicita, oppure disattivarlo in entrambi i bracci.
4. **T3 – separare l'effetto della posa da quello della finestra temporale.**
   - C1: stessi tempi di start, stato protesico perturbato (motor_angle = q, motor_speed = qdot, FSM re-inizializzata).
   - C2: stessa fase su cicli diversi, per esempio 0.38, 1.96 e 3.54 s. L'offset massimo è 4.01 s.

**Un reset con qdot = 0 non è una posa statica bilanciata.** A 1.957 s il piede protesico è in stance e il riferimento biologico ha qdot_ref ≠ 0, quindi il termine Kd·e_qdot genera subito un transitorio. La GRF destra resta quella dinamica di quell'istante, e l'equilibrio è comunque imposto dai residui.

**Confondenti**
- Lo start cambia anche la finestra di IK biologica e di GRF destra.
- Accredito della FSM al primo ciclo.
- Transitori di reset: τ_SEA = 0 con piede a terra, muscoli a 0.01, accelerazione del governor a 0.
- Critic addestrato su un solo start, con la stessa lr bassissima dell'actor.
- Ottimizzazione quasi ferma.
- Runner sincronizzati e σ = 0.01: poca diversità per batch.
- Pipeline del learner che potrebbe differire tra i bracci.
- Corridoio morfologico derivato dall'IK.
- Valutazioni storiche fatte solo a 1.957 s.
- Offset oltre il limite troncati senza avviso.
- Codice del run diverso dal working tree.

## Dubbi e controevidenze

- **Contro l'ipotesi:**
  - il reset è 1 transizione su 500 e ogni episodio copre circa 3 cicli;
  - i return dello stesso start variano tra iterazioni, quindi le traiettorie non sono copie;
  - la curva piatta si spiega più semplicemente con lr 5e-7 e con il critic.
- **Compatibile con l'ipotesi, ma non probante:** 3 stati iniziali, runner sincronizzati, rumore minimo, forte dipendenza dallo start, valutazioni solo su uno start di training.
- **Non verificato:**
  - lo stato FSM al reset per gli start 1.757 e 2.157 s (non c'è un artefatto);
  - l'indipendenza del rumore tra runner;
  - le componenti del reward separate per start (non sono loggate).
