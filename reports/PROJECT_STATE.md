# Stato corrente del progetto

**Ultimo aggiornamento:** 2026-10-29. Fonti: daily fino al
[2026-10-26](daily/2026-10-26_daily-report.md) e indicazioni dell'utente del
2026-10-29.

Questo file si legge a `start_day`. `end_day` lo **riscrive**: non si
accodano righe. Restare sotto circa 150 righe; i dettagli vanno nei report
linkati. TODO completi in [TODO_BACKLOG.md](TODO_BACKLOG.md); comandi e
ambienti in [RUNBOOK.md](RUNBOOK.md).

## Lavoro in corso e prossimo passo

**In corso:** analizzare se best47 gestisce le variazioni della velocità del
passo e risolvere l'incapacità di farlo. I due report di riferimento sono il
[test sulle variazioni di velocità](user/2026-10-26_test_variazioni_velocita_treadmill_best47.md)
e il [test di fermo e ripartenza](user/2026-10-26_test_camminata_fermo_ripartenza_best47.md),
con la [diagnosi del mancato arresto](user/2026-10-26_diagnosi_fermo_ripartenza_best47.md).

Evidenze finora:
- **0,8 → 1,2**: terminazione per penetrazione (28,19 mm) a 5,93 s, durante
  l'accelerazione. Il regime lento non è qualificato. Il ritardo dell'HS
  reale sale da 111 a 274 ms. Con contatti ideali la FSM accetta le cadenze.
- **Fermo**: la protesi non si ferma, perché la policy continua a chiedere
  target oscillanti. La FSM actor non ha uno stato standing e va in `TIMEOUT`
  dopo 2,2 s di stance.
- **Comune ai due**: l'actor non riceve comandi di velocità né di arresto, e
  best47 è allenato solo sulla camminata nominale AB06 (tre offset dello
  stesso cammino).

**Prossimo passo.** Le proposte dei report sono da scegliere con l'utente. Per
il vincolo sui fix strutturali, la soluzione va progettata con design e piano
di test a livelli approvati prima del codice.
- **SPD-T01**: prove mirate con plateau più lunghi e rampe più lente, per
  separare il ritardo del generatore dalla risposta del contatto.
- **STP-T01**: definire intento di arresto, appoggio verificato, stato
  standing nella FSM actor e obiettivo nella reward; poi episodi di
  transizione nel training.

## Policy e artefatti di riferimento

**best47** è la policy di riferimento: actor a 35 feature (stato
protesico/motore, carico, fase, FSM, memoria del controllore) e critic a 84.
Allenata con reward `ex_novo` sulla camminata AB06: 50 update, episodi da 5 s,
t0 = 13,946871 s. Catena FSM: binary V26 → adapter → FSM actor v3.

| Artefatto | Percorso |
|---|---|
| Checkpoint best47 | `Trajectory Generator/runs/training/MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/rl_module_best` |
| Preset ex novo: default del training e configurazione dei rollout best47 | `Trajectory Generator/baseline_MLP/training_exnovo_cfg.yaml` |
| Preset imitativo, solo con `--config` | `Trajectory Generator/baseline_MLP/training_cfg.yaml` |
| Setup del modello AB06 treadmill (stiffness 321/500, driver PI) | `models/AB06/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml` |
| Profilo GRF online e detector HS/TO | `online_GRF/profiles/AB06_SEASEA_stiff321_500_pi_online_full_wrench_residual_tangent_v2.json`, `..._grf_detector_HS-TO.json` |
| Geometria del detector binario | `online_GRF/binary_detector/operational/heel_toe_v25.json` |
| Profilo morphology | `Trajectory Generator/baseline_MLP/morphology_corridor/profiles/ab06_prosthetic_event_warped_mean_std_corridor.json` |
| Harness del lavoro in corso | `Trajectory Generator/validation/speed_transitions_2026_10_26/` |
| Plot delle campagne di ottobre | `plot/speed_transitions_2026_10_26/`, `plot/random_pose_reset_2026_10_13/`, `plot/reset_ik_audit_2026_10_12/`, `plot/tnt_*_2026_10_13/`, `plot/*AB07*_2026_10_0[67]*` |
| Registro dei run storici | `Trajectory Generator/runs/historical_runs.md` |

## Decisioni e vincoli vigenti

Decisioni dell'utente:

- **Nessun lancio senza via esplicito** (G01, D05): download, preparazione di
  modelli, conversioni, rollout e training solo dopo un via, nei suoi limiti.
- **Fix strutturali, non tamponi sulla policy** (dal 2026-08-21): per un
  difetto architetturale servono un design document e un piano di test a
  livelli con gate dichiarati, approvati prima del codice. I livelli sono
  unit, replay di trace, cinematica prescritta, policy
  ([design](plans/2026-08-21_design_fsm_v3_resync.md),
  [piano di test](plans/2026-08-21_piano_test_fsm_v3_resync.md)).
- **Linea corrente: V26 / best47** (D01, D02). La rete unificata 39D è stata
  sospesa il 05/10. `unified_MLP/` e `unified_MLP_39/` sono state eliminate
  come deprecate il 17/10 ([registro](cleanup/2026-10-17_trajectory_removal/README.md)).
- **Baseline dei controllori SEA «morning-best» del 18/05**, riferimento per
  i confronti:
  - cascade in `config.py`: ginocchio Kp_outer 18,85, Kp_inner 29,2,
    Ki_inner 1377; caviglia 47,125, 2,8275, 213;
  - driver PI nel modello AB06 PI: ginocchio Kp 18, Kd 11, Ki 190; caviglia
    11,3, 11, 123;
  - il filtro passa-basso su qdot è stato rimosso il 20/05 e non va
    riproposto come default
    ([sweep](user/2026-05-18_sweep_cascade_ankle_best_config.md),
    [opzione D](user/2026-05-19_optD_lpf_qdot_feedback.md)).
- **Riferimento protesico filtrato a 6 Hz dal simulatore** (dal 2026-06-01):
  la rete genera un riferimento smooth e il simulatore lo filtra. Il flag è
  `enable_pros_ref_lpf`, attivo di default in `osim_trj_cmc_like.py`
  ([report](user/2026-06-01_filtro_riferimento_6hz_lato_simulatore.md)).
- **Niente anatomia né identità del soggetto nell'actor** (D07, D08).
- **Morphology corridor attuale conservato** (D09): non va allargato per far
  passare nuovi soggetti. Domain randomization rinviata (D10).
- **best47 congelato**: le valutazioni non aggiornano i pesi; i nuovi training
  vanno in run separati (V26-T03). Il training usa 12 EnvRunner
  (decisione del 29/10, vedere RUNBOOK).

Fonte di D01–D16: [tabella del 05/10](user/2026-10-06_generalizzazione_multimodello_epic_v26_strategia_vincoli_e_audit.md).

Conclusioni tecniche da non rimettere in discussione senza nuove prove:

- **Il fermo non si ottiene allungando il timeout FSM** o congelando la fase:
  con il timeout a 10 s la pausa diventa una stance di 4,07 s. Il movimento
  viene dalla policy, non dal filtro dei riferimenti.
- **AB07: non aumentare i limiti SEA al posto della diagnosi** e mantenere
  fissa la calibrazione del detector (AB07-P01). La registrazione del
  montaggio ha un effetto causale sul carico.
- **Il reset usa q e qdot dall'IK** con tre stati iniziali distinti: la
  copertura è ristretta e i test non provano un effetto causale sul PPO.

## Struttura della codebase

- Riorganizzazione del 14–20 ottobre: layout in `AGENTS.md`, rimozioni in
  `reports/cleanup/`. I report precedenti al 19/10 citano percorsi storici.
- Ultimo commit `dba42388c` (14/10): la riorganizzazione successiva è ancora
  nel working tree, senza commit.

## Risultati recenti

| Data | Prova | Esito | Report |
|---|---|---|---|
| 06–08/10 | Generalizzazione AB07 | La registrazione del montaggio (allungamento distale 17,579 mm) ha un effetto causale sul carico; pylon 400 mm | [10-08](user/2026-10-08_ab07_anatomia_preparazione_pylon_400mm_e_rollout_v26.md) |
| 12/10 | Reset IK nella rete ex novo | Tre start distinti; cambiare start modifica return, transitorio e penetrazione | [10-12](user/2026-10-12_reset_ik_exnovo_audit_indipendente_e_test.md) |
| 13/10 | Reset pose protesiche casuali | 18/20 recuperi qualificati; CI95% 68–99% condizionale | [10-13](user/2026-10-13_reset_pose_casuali_protesiche_v26_audit_e_test.md) |
| 26/10 | Fermo e ripartenza | Sequenza completata, ma la protesi **non si ferma** | [10-26](daily/2026-10-26_daily-report.md) |
| 26/10 | Velocità 0,8 → 1,2 | Terminazione per `grf_penetration` a 5,93 s | [10-26](daily/2026-10-26_daily-report.md) |

Prove dinamiche native macOS arm64; la verifica live Windows resta aperta.

## TODO prioritari

Testo completo in [TODO_BACKLOG.md](TODO_BACKLOG.md).

- **SPD-T01**, **STP-T01**: velocità e arresto, il lavoro in corso.
- **V26-T04**: avvio autonomo del cammino da fermo.
- **RIK-T01** / **RIK-T02**: confronto PPO sui reset; bootstrap causale.
- **RPR-T01**: causa fisica prioritaria nel wrapper reward V26.
- **V26-T01..T03**, **AB07-P01** / **AB07-P02**: detector, trasferimento
  AB06/AB07, dimensionamento della protesi.

## Note aperte

- I daily del 14, 15 e 26 ottobre sono stati ricostruiti il 29/10. Per le
  pulizie del 16–20 ottobre non esistono report utente né daily: i dettagli
  sono nei registri di `reports/cleanup/`.
- Campagna TNT del 13/10: solo plot e citazione nel commit `dba42388c`, senza
  report utente; l'harness è stato eliminato il 17/10.
- Il registro eredita 276 voci, molte simili o legate a materiali rimossi.
  Una revisione con chiusura motivata è un'attività da concordare.
- Per il layout corrente su Windows servono commit e push della repository SEA
  esterna e le DLL nelle nuove posizioni (daily del 15/10).
