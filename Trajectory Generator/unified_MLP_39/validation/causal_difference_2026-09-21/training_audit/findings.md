# Audit del learner, degli ingressi e della provenienza fisica

Data: 2026-09-21. Analisi dei checkpoint e degli artefatti esistenti; nessun training, rollout o modifica al runtime. V26 di riferimento: `MLP_imitation_native_v26_08-20-2026_june_equiv_100iter`; run attuale: `MLP39_v26_aligned_left_phase_fsmv3_25iter_2026-09-21`, completato fino alla 40.

## Esito

Non emerge una riduzione della capacità della rete, una perdita di ingressi fisici, una deriva della configurazione PPO o una modifica della dinamica fisica di base. Le differenze reali aggiuntive rispetto al target sinistro e alla FSM v3 sono la gestione dei fallimenti fisici, il batch effettivo storico e l'ordine degli ingressi. Nessuna di queste prove identifica da sola la causa del gap recente di completamento.

## Actor, critic e inizializzazione

- I pickle `checkpoint_best/class_and_ctor_args.pkl` dello storico e `checkpoint_last/class_and_ctor_args.pkl` attuale sono stati letti direttamente. Entrambi hanno actor39, osservazione completa88, MLP2×256 tanh e 165.893 parametri addestrabili.
- Nel V26 `model_config.n_full=84` è metadata obsoleto. La `observation_space` reale è88; il modulo costruisce il critic dalla shape, non da `model_config.n_full` (`baseline_MLP/asymmetric_rl_module.py`, metodo `setup`). Non era un critic84.
- Il suffisso39:88 è identico per nomi e ordine fra `rollout_summary.json` V26 e `unified_contract.json`39D. Le quattro diagnostiche `u_abs/u_saturated` sono critic-only anche nello storico, perché oltre il taglio reale a39.
- Gli ingressi target storici2:6 sono ora35:39; gli altri ingressi actor sono permutati. Clock e target hanno inoltre la semantica sinistra deliberata; le features FSM riflettono la v3 deliberata.
- Ricostruendo i due moduli con seed123, tutti i pesi comuni risultano esattamente uguali. Il nuovo `rl_module_initial` coincide con l'inizializzazione seed123.
- La permutazione degli ingressi senza permutare le colonne del primo layer cambia la funzione iniziale per quel seed. **Non riduce la capacità e non dimostra peggiori prestazioni medie**: la permutazione è compensabile nei pesi e la distribuzione iid di inizializzazione è statisticamente invariante.
- La disponibilità prescribed usa un RNG NumPy indipendente. A probabilità1 non oscura canali, non perturba il RNG Torch e non resetta la fisica a ogni iterazione. Il modulo in fase `imitation` non congela né actor né logstd né critic.

## PPO effettivo

Il confronto ricorsivo delle configurazioni PPO serializzate nei checkpoint non evidenzia differenze learner/optimizer/GAE/distribuzione azioni/sampling. Le differenze sono i nomi di classi/env/callback, campi target/FSM e nuovi campi reward disabilitati. `max_delta_rad` rimosso era inerte in modalità absolute.

I log confermano LR1e-4, coefficienteKL0,2, coefficienteentropia0, 10epoche e minibatch512. A esperienza comparabile, V26iter33 ha elaborato3.224 minibatch e 39Diter40 ne ha elaborati3.240: non rimane un deficit sostanziale di aggiornamenti.

| Finestra | V26 29–33 | 39D 36–40 |
|---|---:|---:|
| KL media | 0,010061 | 0,009922 |
| VF loss media | 3,021 | 2,954 |
| Entropia media | 2,0689 | 2,2010 |
| VF explained variance media | 0,1287 | 0,1635 |

L'esplorazione risultante è leggermente più ampia nella nuova rete, ma non deriva da un coefficiente configurato diverso. Questi numeri non dimostrano causalità né un problema numerico del learner.

## Fallimenti fisici e dati raccolti

Il diff contro `68a62cd8` mostra due cambiamenti oltre la FSM:

1. Il superamento della guardia fisica30mm è ora catturato come terminazioneMDP `grf_penetration_hard`, con safety penalty (`osim_trj_cmc_like.py:1868`, `:1926`, `:1946`). Prima, con fail-fast, causava morte del worker.
2. Alcune incoerenze di stato del detector ora terminano come `invalid_binary_event`, invece di lasciare un cursore desincronizzato e provocare una successiva morte del worker (`osim_trj_cmc_like.py:3595`).

Questo spiega sia l'oversampling storico sia una possibile selezione diversa dei frammenti consegnati al learner: un fallimento del worker può far perdere dati e contatori, una terminazione pulita viene appresa come tale. Non è opportuno ripristinare quei bug. Nel tratto recente, tuttavia, il problema dominante sono terminazioni ordinarie a28mm comuni alle due ricette, dunque questa differenza non spiega da sola il gap38,8%/88,4%.

Controllo specifico della finestra V26 29–33: il `faulthandler.log` la colloca fra le08:34:34 e le09:12:38 del20agosto; i cinque batch sono esattamente4096 e le durate regolari, circa7m35–7m40. Non esistono contatori di salute worker nei TensorBoard salvati. La sessioneRay originale `/private/tmp/ray/session_2026-08-20_03-30-49_168394_90281` conserva solo i socket: i log sono stati ripuliti; la cartella `rllib` del run è vuota. Non si può attestare zero morti worker in quella precisa finestra. L'evidenza indiretta è compatibile con campionamento regolare; **il possibile bias dei crash storici non va usato come spiegazione del valore88,4% nella finestra recente**.

## Provenienza fisica

Il file [physical_provenance.json](physical_provenance.json) contiene i digest correnti, il confronto col commit storico68a62cd8 e il confronto col lock storico del9agosto `baseline_MLP/validation/h0_primary_split_v12r1_autonomy_recovery_execution_lock.json`.

Risultano byte-identici al commit storico: `config.py`, `model_loader.py`, `kinematics_interpolator.py`, `prosthesis_controller.py`, `inverse_dynamics.py`, `static_optimization.py`, `outer_loop.py`, `online_grf.py`, `setup_io.py`, setupXML, modelloOSIM, sorgentiSEA C++ e profili detector. In `simulation_runner.py` l'unica differenza rispetto al commit è la classe d'eccezione della guardia30mm, senza cambiamento della soglia o delle equazioni.

Per gli asset non versionati in quel commit, coincidono gli hash archiviati nel lock di agosto:

| Asset | SHA256, prefisso |
|---|---|
| IK `IK_results_AB06_SEASEA.mot` | ce4a948fd8f01f34 |
| GRF `AB06_SEASEA_GRF_FullSpan.mot` | 6b84cce4abd6b313 |
| `ExternalForces.xml` | a2fb6460fc0f8ae9 |
| `CMC_Actuators.xml` | 238792c0024c4052 |
| SEA dylib `libSEA_Plugin_BlackBox_mCMC_impedence_ff.dylib` | 77390d0f74055fb3 |
| OnlineGRF dylib | 5597a59a5368825f |
| Profilo forze `online_full_wrench_residual_tangent_v2` | 09e04ab94954703d |

L'hash del profilo forze è inoltre esplicitamente registrato nell'audit del20agosto `reports/user/2026-08-20_audit_equivalenza_giugno_per_fresh_run_nativo_v26.md`, sezione derive: il vecchio nome `grf_correct_magnitude` e `tangent_v2` erano byte-identici. Non è uno shift di dominio fisico.

Setup, profili applicati e di detection, routing left-online/right-prescribed sono gli stessi nei checkpoint e nei summary. Il reset fisico a12,99s è già confrontato in `validation/v26_config_alignment_2026-09-21/start_alignment.json`:18 campi fisici uguali, delta massimo0. L'assenza di un file dati diverso è quindi sostenuta sia dai digest sia dallo stato di reset.

Le versioni correnti torch2.10.0, ray2.55.1, numpy2.2.6, opensim4.5.2, PyYAML6.0.3 e gymnasium1.2.2 coincidono con il fingerprint archivioF0 del22agosto; il checkpoint V26 attesta direttamente Ray2.55.1. Il fingerprintF0 era un inventario retrospettivo, non un'attestazione delle librerie importate da ogni processo.

## Data del rollout storico e limiti

Il confronto cinematico precedente usa un rollout **prodotto il20agosto**, non rigenerato a settembre: `watchdog_state.json` riporta timestampUnix1787241977,129685, equivalente a20agosto2026 alle18:06:17Europe/Rome. I timestamp dei file summary18:06:16,97 e trace18:06:17,13 concordano. Il training V26 finì alle17:57:28.

I digest e la storia Git escludono una deriva persistente negli asset esaminati. Non costituiscono una prova dei byte nativi effettivamente mappati da ciascun processo storico, né possono escludere modifiche transitorie non archiviate. Non esiste una chiusura completa per-processo del run del20agosto. La conclusione corretta è che **non abbiamo evidenza di una modifica della fisica o dei dati sorgente**; non una garanzia retrospettiva assoluta.

## Verifiche e file prodotti

Lettura dei checkpoint e dei contratti osservazioni; costruzione offline dei due moduli con seed controllato; confronto dei log learner; diffGit; hash degli asset; confronto con manifest storici; timestamp del rollout storico. Prodotti solo questo documento e `physical_provenance.json`; nessun runtime modificato e nessun episodio simulato.
