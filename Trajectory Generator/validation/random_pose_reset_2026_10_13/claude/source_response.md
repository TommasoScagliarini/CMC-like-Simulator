**Audit indipendente — reset da pose protesiche casuali (V26C best47, AB06)**

## Verdetto

Il bootstrap coerente non reintroduce l'IK protesica, né al reset né nel primo segmento servito. Prima del run però vanno sistemate tre cose:
1. Il bootstrap probe ha un probabile bug sulla forma e sul clipping dell'azione.
2. I failure vengono classificati male e le loro tracce si perdono.
3. "≥2 cicli" e "primo HS ≤3 s" sono criteri al limite o asimmetrici, e nessun criterio misura davvero il recupero del gait.

## 1. Catena del bootstrap (verificata)

- **Reset del governor.** `_initialise_episode` chiama `kin.clear_segment()` (`osim_trj_cmc_like.py:2565`). L'adapter imposta il governor a q=posa, v=a=0 (`random_pose_reset.py:60-67`).
  - Il percorso legacy eliminato è questo: senza adapter il filtro verrebbe inizializzato dall'anchor (`osim_trj_cmc_like.py:707-719`). Senza segmento, quell'anchor viene da `get`, che ricade sull'IK (`:829-834`).
- **Reset dello stato.** `reset_to_time` legge `_kin.get(t)` una sola volta (`simulation_runner.py:346`).
  - motor_angle = posa, motor_speed = qdot = 0 (`:1817-1842`); stati plugin a zero (`:1789-1794`); PI azzerato (`:375`).
  - L'unico altro uso di `_kin` è all'inizio di ogni finestra (`:865`), sempre dentro il segmento.
- **Primo segmento.** `_last_policy_endpoint` = posa (`osim_trj_cmc_like.py:2596-2600`) ed è l'anchor dello slew limiter (`:2691`). Anchor di continuità e derivata vengono dall'adapter (`:2683-2704`).
  - Il modello butterworth3 usa solo `values[-1]` (`:736`) più lo stato del filtro. Il riferimento servito è quindi C2 a partire da (posa, 0, 0).
  - Questo è verificato end-to-end in `set_segment` (`random_pose_reset.py:100-101`).
- **Osservazione dell'attore.** Nessuna feature IK protesica:
  - gait clock disabilitato (`training_cfg.resolved.yaml:62`);
  - `include_imitation_target_observation` è falso in modalità ex-novo (`env_factory.py:192-194`);
  - il target IK sta solo nel suffisso privilegiato (`osim_trj_cmc_like.py:3095-3100`).
- **Carico e FSM al reset.** Il carico iniziale osservato è 0 (`:3158`, `:3360-3364`). L'FSM viene inizializzata dai bit geometrici a t0 (`:3203-3268`) senza HS sintetico (`prosthetic_phase_fsm.py:243-279`, `binary_phase_fsm.py:116`).
- **Fuori segmento.** L'adapter solleva un errore invece di ricadere sull'IK (`random_pose_reset.py:75-76`). È corretto, ma va classificato come errore dell'harness.

**Caveat**
- Il controllo del riferimento servito al reset (`:226-229`) è tautologico: l'adapter restituisce `self.pose` per costruzione (`:71-72`).
- Per il caso nominale manca un assert su `base.t == t_start+offset` (`:180-182`).

## 2. Difetti dell'harness

1. **Bug del probe.** `_deterministic_action(..., env.action_space.shape)` restituisce la forma `(2,)` del wrapper appiattito (`env_factory.py:129-133`), non clippata. `_action_to_segment` invece indicizza `action[:, j]` (`osim_trj_cmc_like.py:2677`).
   - Atteso un `IndexError` per ogni caso ammesso. A quel punto il worker restituisce 1 e il loop del supervisor si ferma (`random_pose_reset.py:444-445`).
   - Correzione: fare reshape e clip come `FlattenClipAction.action` (`env_factory.py:135-137`).
2. **Il post-processing sovrascrive l'esito fisico.** `render` usa `mkdir(exist_ok=False)` e solleva `RuntimeError` (`:256`, `:270`). Nessuno dei due è catturato da `(KeyError, ValueError, FileNotFoundError)` (`:365`), quindi lo status diventa `harness_or_native_failure`.
   - Correzione: tenere separati `physical_status` e `postprocessing_status`.
3. **Failure fisici etichettati come errori dell'harness.** Con `fail_fast=True` (`rollout_eval.py:508`) diventano eccezioni:
   - la guardia di forza a 5 BW (`simulation_runner.py:1652-1656`);
   - le static optimization non risolvibili e i crash nativi.
   - Effetti: il batch si ferma, e `rollout_summary` e `policy_trace` si perdono, perché vengono scritti solo a fine loop (`rollout_eval.py:1024-1033`).
   - Regola da preregistrare: un'eccezione dopo almeno uno step è un failure fisico, con sottotipo, e non viene mai escluso.
4. **Tracce insufficienti.**
   - `returned_rewards.jsonl` (`random_pose_reset.py:315-327`) non contiene azione, `phase_fsm`, eventi, bit del detector né diagnostica del governor. Serve un JSONL per step con flush; nell'except, dump di `runner.last_step_info`, payload FSM e tipo di eccezione.
   - Il segmento fallito è censurato: l'FSM non avanza (`osim_trj_cmc_like.py:3568-3576`) e i campioni binari parziali vanno persi. Solo la guardia a 30 mm produce `terminal_contact` (`:1872-1879`).
   - Eventi e conteggi vanno quindi trattati come censurati all'ultimo boundary completato.
5. **Selezione non meccanizzata.**
   - `--count` vale 8 di default (`:400`) e nessun codice implementa "primi 20 ammessi".
   - Il probe usa la config storica (`:340-343`), il run quella di `rollout_eval` (`rollout_eval.py:466-541`).
   - Correzione: congelare la lista dei 20 ID e l'hash dei gate prima del run, e riverificare nel run che il gate dia lo stesso esito.
6. **Escursioni.** `analyze` calcola l'escursione su tutto il record (`frozen_reset_tests.py:435`), quindi include il transitorio dalla posa iniziale. Va calcolata per ciclo.

## 3. Confondenti

- **Biologia guidata dall'IK con reserve.** Il bacino non può cadere. Nei casi AIR il supporto sinistro mancante lo assorbono le reserve. Il test quindi non misura l'equilibrio.
- **Azioni assolute + slew 2.5/2.0 rad/s + governor a 4 Hz.** Anche una sequenza di azioni open-loop tenderebbe a convergere.
  - Controllo raccomandato, solo lato harness: replay open-loop delle azioni del nominale coerente a partire dalle stesse pose.
  - Se il replay recupera allo stesso modo, il merito è della parametrizzazione e non del feedback della policy.
- **Dominio rispetto all'IK.** Il range IK è circa knee [-1.06,-0.13] e ankle [-0.13,0.40] (`osim_trj_cmc_like.py:210`), quindi circa il 62% del rettangolo è fuori dal box IK.
  - Knee a -0.03 dista solo 0.03 rad dalla terminazione `joint_divergence` a 0.0 (`:314-318`).
- **Gate a 27 mm.**
  - Riferimenti: inviluppo di contatto sano circa 11.5 mm (`:293-295`), soglia di penalità 20 mm, terminazione 28 mm (`training_cfg.resolved.yaml:47-48`).
  - Restano 1 mm di margine e si ammettono stati fisicamente irrealizzabili.
  - Proposta: stratificare AIR / ≤12 mm / 12–27 mm.
  - In più, a t0 sono ammesse forze fino a 5 BW che la policy non vede (carico osservato = 0).
- **Detector ≠ contatto di forza.** Il detector usa punti e un piano (`binary_phase_detector.py:265-313`), la forza usa sfere (`online_grf.py:614-616`).
  - L'FSM parte dal detector, il carico dalla forza.
  - Definire AIR con entrambi; nota che un contatto solo di punta (TOE-only) viene inizializzato come stance.
- **Ambito ristretto.** Un solo istante (HS sinistro), velocità zero, molla SEA scarica, policy deterministica, una sola piattaforma.

## 4. Criteri

**Problemi dei criteri proposti**
- **≥2 cicli in 5 s è al limite.** Il periodo misurato è 1.669 s (`frozen_manifest.json:55-57`).
  - Da una partenza in contatto, nominale compreso, il primo HS valido cade a circa 1.67 s e non può chiudere un ciclo (`prosthetic_phase_fsm.py:1209-1219`).
  - Se il periodo resta costante, il terzo HS cade a circa t0+5.007 s, cioè dopo la fine dell'episodio.
- **Asimmetria a favore dei casi AIR.** I casi AIR accettano subito un HS da WAIT_HS (`:1186-1194`). Lo stesso vale per "primo HS ≤3 s".
  - Rimedio: orizzonte di 7.0 s (t_end = 21, massimo 7.05 s; l'attore non ha feature temporali), oppure soglie relative al nominale.
- **RMSE actual–served misura il controllore, non il gait.** Anche la soglia FSM di 0.12 rad di escursione (`training_cfg.resolved.yaml:183`) è troppo permissiva.

**Proposta a livelli**
- **L0 – integrità:** le asserzioni del bootstrap passano.
- **L1 – completion:** 500 step senza terminazione né eccezione.
- **L2 – sicurezza:** penetrazione <28 mm sui campioni a 1 ms, forze, `joint_divergence`.
- **L3 – eventi:** dopo il primo HS devono essere a zero `invalid_event_count`, `resync_count`, `hs_cancelled_count`, `timeout_exceeded` e `cycle_degraded`.
  - Il contatore env `_binary_phase_invalid_event_count` non è pubblicato (`osim_trj_cmc_like.py:3648-3652`): usare il payload FSM più `end_reason`.
- **L4 – primario, recupero = convergenza al nominale coerente sulla stessa base temporale:**
  - HS/TO dopo 2 s entro ±40 ms da quelli del nominale;
  - RMSE knee/ankle dell'ultimo ciclo sotto una soglia preregistrata, calibrata sulla differenza tra nominale coerente e legacy;
  - ROM per ciclo ≥0.8× quello nominale;
  - deviazione dal nominale decrescente nel tempo.
- **L5 – costi:** impulso delle reserve nel primo secondo, saturazione SEA, clipping, frazioni di slew/governor attive, RMSE ≤2°.
- **Gating:** se il nominale coerente non supera tutti i livelli, l'esperimento si ferma.

## 5. Inferenza da N=20

- **Intervalli (Clopper–Pearson 95%).**
  - 20/20 → [83.2%, 100%], cioè probabilità di failure ≤13.9% (unilaterale).
  - 19/20 → [75.1%, 99.9%].
  - Per affermare ≥90% servono 29/29; per ≥95%, 59/59.
- **Risoluzione spaziale.** Con 20 punti la spaziatura tipica è circa 0.25 rad: non basta a mappare il bacino di attrazione. Riportare la mappa successi/failure per strato e il tasso di scarto del gate.
- **Conclusione ammissibile.** Solo se il nominale passa e tutti, o quasi, i casi superano L1–L4:
  > "Per AB06, reset all'HS sinistro a 13.947 s, biologia IK con reserve, velocità zero, pose uniformi nell'insieme ammesso dal gate: la policy deterministica riconverge al ciclo nominale entro T s, con probabilità ≥~86%."
- **Non dimostrato:**
  - altre fasi del passo, velocità o precarico SEA non nulli;
  - equilibrio, avvio da fermo, disturbi, rumore dei sensori, altri soggetti;
  - pose scartate dal gate;
  - stabilità oltre ~3 cicli;
  - merito del feedback della policy (senza il controllo con replay open-loop);
  - riproducibilità tra macOS e Windows.

**Fonti.** Oltre all'elenco ho consultato in modo mirato `frozen_reset_tests.py` e `frozen_manifest.json` del 12/10 (l'helper che l'harness importa), `env_factory.py`, `config.py`, `binary_phase_fsm*.py`, `binary_phase_detector.py`, e setup XML/.osim. Non ho letto né `test_bootstrap.py` né conclusioni o risultati Codex. Non ho eseguito nulla: il bug del probe è dedotto dal codice e va confermato con un probe.
