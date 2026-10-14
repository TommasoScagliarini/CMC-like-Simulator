# Diagnostiche finali random-pose AB06 best47

Data: 13 ottobre 2026. Analisi descrittiva dei file salvati; criteri congelati invariati, nessun training o nuovo rollout, nessuna lettura Claude.

Il risultato preregistrato è **18/20 recuperi**, due terminazioni ai guard e zero casi irrisolti. CI esatto 95%:68,30–98,77%; lower bound one-sided 95%:71,74%. La probabilità riguarda il dominio geometricamente ammesso in questo banco.

Fra i 18 PASS il return varia da **-109.819 a 61.381**, mediana **26.214**; **7/18** sono negativi. Il controllo coerente ha return 27.317. Il recupero gait e il return misurano aspetti distinti.

I sette PASS negativi partono tutti con detector AIR e primary contact False; completano tre cicli validi. Le somme della morphology loss già registrata sono 12,444–29,837, contro 2,956 del controllo, con pesi knee/ankle 5/5. È un confronto di diagnostiche registrate, senza ricostruire contributi additivi del return o attribuire causalità al training.

| Gruppo iniziale random | N | PASS | Primo HS confermato nei PASS [s], min–mediana–max | Cicli nei PASS |
|---|---:|---:|---|---|
| primary_contact_False__detector_TOE | 3 | 3 | 1.650–1.650–1.650 | {2: 3} |
| primary_contact_False__detector_HEEL | 2 | 2 | 1.649–1.649–1.649 | {2: 2} |
| primary_contact_True__detector_HEEL | 3 | 2 | 1.649–1.649–1.650 | {2: 2} |
| primary_contact_False__detector_AIR | 10 | 9 | 0.130–0.373–0.428 | {3: 9} |
| primary_contact_True__detector_TOE | 2 | 2 | 1.649–1.649–1.650 | {2: 2} |

Primary contact False riguarda 15/20 pose, detector AIR 10/20; cinque pose primary False hanno heel/toe detector ON. Le piccole forze iniziali residue osservate nei primary False sono nell’intervallo 1.37e-05–0.000506N e non dimostrano un appoggio significativo. Non confondere i due piani di contatto.

Nell’ultimo secondo4–5s, actual rispetto al controllo sul medesimo clock: knee RMSE 0,01154–2,88643° (mediana 0,24606°), ankle 0,00811–0,99321° (mediana 0,10185°). I massimi sono random_016. Le traiettorie non convergono esattamente al nominale; questo confronto è distinto dal gate actual-served<=2°.

Le coordinate biologiche actual restano vicine alla risposta nominale: massimo RMSE 4–5 s 0,07556° (hip_flexion_l, random_016) e 0,07722 mm (pelvis_ty, random_016). Sono dinamicamente libere con tracking IK/muscoli/reserve, non prescritte cinematicamente. Inoltre la risposta nominale stessa conserva errori di tracking IK: dopo 0,5 s ankle_r≈3,718°, subtalar_r≈2,452° e mtp_r≈13,968°. La somiglianza al controllo non certifica tracking biologico perfetto.

| Costo reserve nei 18 PASS | Startup 0–0,5 s massimo / controllo | Ultimo 4–5 s massimo / controllo |
|---|---|---|
| Norma reserve rotazionali biologiche, RMS [Nm] | 100.58/94.39 = 1.065× (random_020) | 97.33/96.39 = 1.010× (random_018) |
| Norma reserve rotazionali biologiche, Picco [Nm] | 169.19/149.31 = 1.133× (random_020) | 176.16/173.66 = 1.014× (random_016) |
| Norma forza residuale pelvis, RMS [N] | 386.88/227.28 = 1.702× (random_023) | 118.17/105.39 = 1.121× (random_016) |
| Norma forza residuale pelvis, Picco [N] | 729.24/369.71 = 1.972× (random_023) | 278.33/249.16 = 1.117× (random_016) |

RMS e picco della forza residuale pelvis possono quindi quasi raddoppiare all’avvio, mentre i costi nell’ultimo secondo sono più vicini al controllo. ForzeN e coppieNm restano separate; i failure incompleti non sono inclusi nei rapporti di finestre complete.

Clipping: soltanto random025, **13/14 return** sul knee, raw massimo 1,05793 normalizzato; nessun clipping ankle e nessun clipping nei 18 PASS.

**random006:** guard primario di penetrazione a 30,12994 mm, nascosto dal nome finale morphology nel summary. Conservati 32 return/reward, 31 segmenti completati e 9 sample nativi parziali; il classificatore V2 riconosce il failure fisico documentato senza inventare eventi.

**random025:** stop a 0,14 s per guard operativo knee upper 0; actual terminale0.02503° (0.0004368697rad), served-0.0033123325rad. È un piccolo overshoot del limite operativo, entro il range dichiarato del modello ±90°, non evidenza di divergenza catastrofica o violazione ROM del modello. Resta un FAIL preregistrato; continuare o allentare il guard non è stato provato.

Fonti: `aggregate_results.json`, `compact_diagnostics.json`, summary/trace/metrics dei rollout finalizzati, receipt025 e modello OpenSim; path e SHA256 completi in `diagnostics_summary.json`. Non sono cambiati criteri, harness, pesi o dati numerici.
