# Confronto ibrido profili GRF online per fase protesica

## Problema

Dopo la validazione degli eventi HS/TO, restava da chiarire se il profilo `grf_detector_HS-TO`, scelto per il timing della fase protesica, potesse sostituire anche il profilo storico `grf_correct_magnitude` come sorgente fisica della GRF online nel setup CMC-like ibrido.

Il dubbio era importante per il design della reward task-based: una cosa e usare un segnale come sensore di contatto/fase, un'altra e usarlo come forza applicata alla dinamica. Il profilo storico era stato scelto per motivi dinamici e di reserve, mentre il profilo event-tuned era stato selezionato per riconoscere meglio heel strike e toe off.

## Strategia

E stato eseguito un confronto A/B CMC-like breve ma controllato:

- finestra temporale: `11.99-13.99 s`;
- stesso setup XML: `models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml`;
- modalita GRF: `online_sensor`;
- lato sinistro/protesico: prescribed GRF disabilitata e online GRF applicata;
- lato destro: prescribed GRF mantenuta in dinamica;
- run A: `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json`;
- run B: `online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json`.

Sono stati confrontati:

- eventi HS/TO sinistri rispetto alla GRF prescribed;
- contact IoU/F1;
- impulso verticale e picco Fy;
- reserve/residual effort;
- coppie SEA knee/ankle;
- penetration, slip e moment flip.

## Soluzione / Risultati

Entrambe le simulazioni sono terminate con `status=complete`.

Nella finestra corta analizzata entrambi i profili passano il gate eventi con il detector raccomandato per reward/fase:

- threshold basso: `15 N`;
- confirmation threshold HS: `120 N`;
- durata minima contatto: `0.03 s`;
- smoothing causale: `0.10 s`.

La differenza di timing rimane comunque favorevole a `grf_detector_HS-TO`:

| metrica | grf_correct_magnitude | grf_detector_HS-TO |
|---|---:|---:|
| HS matched/ref | 1/1 | 1/1 |
| TO matched/ref | 1/1 | 1/1 |
| TO max error | 53 ms | 2 ms |
| Contact F1 | 0.961 | 0.990 |
| Contact IoU | 0.925 | 0.980 |

Sul piano dinamico, invece, `grf_detector_HS-TO` non e un sostituto diretto del profilo storico:

| metrica | grf_correct_magnitude | grf_detector_HS-TO | rapporto prelim/residual |
|---|---:|---:|---:|
| contatti totali/sinistri | 20/8 | 4/2 | - |
| impulso verticale sinistro | 659.5 Ns | 488.4 Ns | 0.74 |
| impulso sinistro / prescribed | 0.869 | 0.643 | 0.74 |
| picco Fy sinistro | 709.8 N | 434.1 N | 0.61 |
| tau reserve norm p95 | 325.6 | 433.3 | 1.33 |
| root reserve norm p95 | 318.3 | 433.3 | 1.36 |
| pelvis ty reserve p95 | 304.7 N | 400.1 N | 1.31 |
| SEA ankle tau_spring p95 | 20.9 Nm | 28.7 Nm | 1.37 |
| SEA knee tau_spring p95 | 54.2 Nm | 34.5 Nm | 0.64 |
| penetration max sinistra | 0.0175 m | 0.0394 m | 2.26 |
| moment flip | 0 | 0 | - |

## Interpretazione

Il test conferma una separazione concettuale utile:

- `grf_correct_magnitude` resta piu adatto come profilo dinamico per sostenere la protesi nella simulazione ibrida, perche fornisce piu impulso verticale e richiede meno reserve.
- `grf_detector_HS-TO` resta piu adatto come proxy sensoriale per fase/contatto, perche identifica il TO in modo molto piu vicino alla reference e produce una maschera di contatto piu pulita.

Quindi non conviene sostituire automaticamente il profilo storico nel simulatore. La scelta piu solida, per ora, e:

- usare il profilo dinamico storico per l'applicazione fisica della GRF online in CMC-like;
- usare il segnale event-tuned, o un detector equivalente basato solo su contatto/HS/TO, per reward gating e osservazioni actor;
- non usare la magnitudo GRF event-tuned come reward fisica assoluta.

Questo e coerente con il vincolo di deployment: l'actor finale dovrebbe vedere solo informazione di contatto/evento/fase protesica, non una misura completa e perfetta della wrench GRF.

## File creati o modificati

Creati output di test:

- `results/hybrid_profile_ab_grf_correct_magnitude/`
- `results/hybrid_profile_ab_grf_detector_HS-TO/`
- `results/hybrid_profile_ab_comparison/report.md`
- `results/hybrid_profile_ab_comparison/summary.json`
- `results/hybrid_profile_ab_comparison/left_fy_events.png`
- `results/hybrid_profile_ab_comparison/dynamics_comparison.png`

Creato questo report:

- `reports/user/2026-06-26_confronto_ibrido_profili_grf_online.md`

Non sono stati modificati reward, training config, actor observation space o codice del simulatore per effetto di questo test.

## Test e verifiche eseguite

Comandi principali eseguiti:

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python main.py \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml \
  --grf-mode online_sensor \
  --online-grf-applied-side left \
  --online-grf-profile online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_correct_magnitude.json \
  --t-start 11.99 \
  --t-end 13.99 \
  --output-dir results/hybrid_profile_ab_grf_correct_magnitude \
  --log
```

```bash
/opt/anaconda3/envs/envCMC-rllib/bin/python main.py \
  --setup models/AB06_SEASEA_Threadmill/AB06_SEASEA_stiff321_500_pi_setup.xml \
  --grf-mode online_sensor \
  --online-grf-applied-side left \
  --online-grf-profile online_grf_profiles/AB06_SEASEA_stiff321_500_pi_grf_detector_HS-TO.json \
  --t-start 11.99 \
  --t-end 13.99 \
  --output-dir results/hybrid_profile_ab_grf_detector_HS-TO \
  --log
```

Verifiche:

- entrambe le run hanno prodotto `sim_output_run_status.txt` con `status=complete`;
- entrambe le run hanno prodotto `sim_output_online_grf.sto`;
- il confronto e stato calcolato su stessa finestra e stesso timestep;
- il JSON finale `results/hybrid_profile_ab_comparison/summary.json` e stato sanitizzato senza `NaN`;
- il report sintetico del confronto e disponibile in `results/hybrid_profile_ab_comparison/report.md`.

## Raccomandazione operativa

Per la reward task-based ex-novo, procedere con una doppia lettura:

1. profilo dinamico storico per la fisica della simulazione;
2. detector/profilo event-tuned per fase protesica, HS/TO, stance/swing e gating dei termini reward.

I termini reward 1-2 devono quindi essere formulati come reward/penalty di contatto e fase, non come ottimizzazione della magnitudo assoluta della GRF online.
