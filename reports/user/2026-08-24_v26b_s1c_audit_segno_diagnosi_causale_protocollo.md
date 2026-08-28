# V26B — Audit di segno/target, diagnosi causale e protocollo S1C (rev3v)

**Ambito:** solo audit, protocollo, test statici e report. **Nessun fit, rollout, collection, probe o promozione.** A2 resta NON-DEPLOYABLE, σ resta irrisolta. Ogni numero qui sotto è **ri-derivato dagli artefatti congelati** dal tool di audit, non asserito: sette check di riproduzione passano fail-closed.

## 1. L'audit di segno: stavamo misurando la qualità contro il segnale sbagliato
Nel sistema convivevano **due riferimenti diversi**, ed è questo il nodo.

| Segnale | Che cos'è | Ankle: min | frac. negativa |
|---|---|---|---|
| **Target di training** `u_IK` decodificato (`q_ankle = 0,7·a`) | lato **protesico**, AB06 IK | **−0,15516** | **19,4 %** (73 delle 380 righe di training) |
| **Riferimento di reporting** `cache targets[:,2]` | **arto sano controlaterale** | **+0,08647** | **0,0 %** |

Distanza fra i due: **RMSE 0,13480** sulla caviglia (0,09596 sul ginocchio). Non sono due viste dello stesso target: sono **segnali diversi**.

**Perché `sign_agreement` valeva 1,0 con la caviglia sempre positiva.** La metrica nel receipt era `mean(sign(pros_ankle) == sign(healthy_ankle))`. Il riferimento sano è positivo sul **100 %** delle righe e la caviglia protesica realizzata è positiva sul 100 % → il confronto è **degenere** e restituisce 1,0 per costruzione. **Non ha mai testato la plantarflessione.** La metrica è ora dichiarata **VOID** quando il riferimento è sign-degenere, e il tool la restituisce come tale con la diagnosi di degenerazione allegata.

**Inseguivamo il target sbagliato o la coordinata sbagliata?** Il **training** usava la coordinata giusta e il segnale giusto: il target protesico IK, che **contiene** il tratto negativo. Era il **reporting** a usare un'altra coordinata (l'arto sano) per giudicare la qualità. E la conseguenza è meccanica, non accidentale: S0D dista **0,04763** dal riferimento sano mentre il target IK ne dista **0,13480**; quindi **ottimizzare verso u_IK allontana dal riferimento sano per costruzione**. A2 è passata da 0,04763 a 0,06772 esattamente per questo. La "regressione qualitativa" di A2 non è un difetto del fit: è l'effetto di ottimizzare verso un riferimento e giudicare con un altro.

Regola d'ora in poi: **qualità = protesico vs IK protesico, nella convenzione protesica** (negativo = plantarflessione); il segnale sano è **solo** una diagnostica di simmetria e non può mai essere riferimento di ottimizzazione né di accordo di segno.

## 2. Diagnosi causale (evidence-based)

### 2.1 Perché il guadagno offline non si trasferisce
Nello **spazio delle azioni** il trasferimento è **buono**: A2 ha chiuso il 19 % (ginocchio) e il 15 % (caviglia) del gap verso u_IK sui propri stati visitati, coerente con l'holdout offline (18,3 % / 13,8 %).

Nello **spazio degli angoli articolari** il trasferimento è **asimmetrico**, misurato contro il riferimento corretto:

| RMSE vs IK protesico | S0D | A2 |
|---|---|---|
| ginocchio | 0,12639 | **0,12279** (migliore) |
| caviglia | 0,13458 | **0,15229** (peggiore) |

Sulla caviglia il guadagno non solo non si trasferisce: **si inverte**.

### 2.2 Perché la caviglia resta positiva — la prova
1. **La policy comanda già plantarflessione.** S0D comanda `q_ankle` fino a **−0,49494** sull'11,8 % delle righe, **più negativo del target IK stesso** (−0,155). A2 fino a −0,28817 sull'11,0 %.
2. **Il comando non arriva mai al giunto.** `pros_ankle_angle_served_ref` non è **mai** negativo: min **+0,01600** (S0D) e **+0,03170** (A2), frazione negativa 0,0 % in entrambi.
3. **Il giunto segue la reference servita**, non il comando: errore di inseguimento medio < 0,01 rad.
4. **Sulle righe che contano**: nelle 46 righe in cui S0D comanda `q_ankle < −0,05`, la reference servita media è **+0,1918** e l'angolo realizzato **+0,1906**. Il comando è semplicemente **non onorato**.
5. **Non è saturazione dell'attuatore**: `pros_ankle_angle_sea_u` resta entro |u| ≤ 0,052 e non sfiora mai 1,0. Il blocco è a monte, nel percorso reference-governor/slew (slew caviglia 2,0 rad/s = 0,02 rad per step da 10 ms).
6. **Prova finale**: sulle 97 righe in cui il target IK è negativo, l'angolo realizzato è passato da **+0,1433 (S0D) a +0,1743 (A2)**. Cioè **proprio sulle righe di spinta dove si vuole plantarflessione, A2 è più dorsiflessa di S0D**.

**Meccanismo unificante:** solo *metà* della variazione di comando sulla caviglia è fisicamente realizzabile — la metà positiva passa, la metà negativa viene assorbita dal percorso di reference. Una variazione simmetrica del comando produce quindi una variazione **asimmetrica** dell'angolo, e la caviglia realizzata deriva verso l'alto. Questo spiega insieme il §2.1 e il §2.2.

### 2.3 Blocker dichiarato
**Un'escursione negativa della caviglia realizzata non è ottenibile con alcuna modifica della policy sotto la configurazione di produzione congelata.** Persino un comando di −0,49 produce una reference servita di +0,19. Preregistrare un gate «ankle ≤ −0,03» sull'angolo realizzato significherebbe preregistrare un gate **irraggiungibile**. È un risultato di impianto/pipeline, non di apprendimento, e il suo rimedio sta nella configurazione di produzione — che questo stadio non può toccare. Va deciso da te, non aggirato da me.

Nota collegata: il campo `pros_ankle_angle_imitation_target_phase` è identicamente zero su **tutte** le tracce v3 della catena (S0D, R2I, S1A, A2), quindi la finestra di fase B3 non è comunque valutabile allo stato attuale.

## 3. Protocollo S1C (preregistrato, nessuna esecuzione)
**Init: esclusivamente S0D** `481dd0d2…`. A1–A6 e S1A mai come init, source o ancora. σ **irrisolta e non operativa** in ogni stadio.

**Ordine degli stadi:** `S1C-0` probe di fattibilità (offline, nessun fit/rollout) → **tua revisione** → `S1C-1` fit (solo se S1C-0 dice REACHABLE) → **tua revisione** → gate closed-loop con token proprio.

**S1C-0 — probe di fattibilità.** Domanda: sotto il percorso di reference congelato, esiste **qualche** sequenza di comandi ammissibile che porti la caviglia realizzata ≤ −0,03 nella finestra B3? Metodo: raggiungibilità offline sulle tracce registrate, propagando la dinamica della reference servita (filtro 6 Hz + slew 2,0 rad/s, entrambi congelati) sotto il comando più aggressivo ammissibile, e confronto dell'inviluppo raggiungibile con il requisito. Usa **solo dati registrati e parametri congelati**; nessun episodio eseguito. Esiti: **REACHABLE** → S1C-1 può preregistrare un gate vincolante sulla caviglia negativa; **UNREACHABLE** → il requisito viene **escalato a te come questione di configurazione di produzione** e nessun fit viene speso contro un obiettivo irraggiungibile.

**S1C-1 — griglia finita di 3 candidati** (solo se REACHABLE): rapporto ancora/task **r = 5** fissato (l'unico sopravvissuto alla gerarchia rev3t), 300 epoche, batch 256, Adam lr 1e-4, seed 2026, deterministico, clock azzerato, logstd bit-identica, ancora parametrica July 1e-5. **Varia solo** un peso **localizzato in fase** `w ∈ {2, 4, 8}` sulle righe di spinta (quelle con `q_ankle_IK < 0`), applicato **dentro il solo ruolo task**. Motivazione: la MSE uniforme per riga spende il budget limitato in modo uniforme, mentre il requisito di plantarflessione è concentrato sul 19,2 % delle righe; `w` localizza il budget senza toccare `r` né alcuna soglia. Ricerca **chiusa**.

**Quality gate preregistrati** (convenzione protesica, riferimento protesico IK):
- **G1 preservazione della deambulazione**: i sette gate FSM/sicurezza di rev3u, invariati e vincolanti.
- **G2 non-regressione vs S0D sul target IK**: `RMSE(pros_q, q_IK) ≤` baseline S0D **[0,12639 ginocchio, 0,13458 caviglia]** per giunto. **È il gate che mancava**: A2 lo fallirebbe sulla caviglia (0,15229 > 0,13458).
- **G3 non-regressione sulla simmetria**: `RMSE(pros_q, healthy_q) ≤ 1,05 ×` baseline S0D **[0,12057, 0,04763]** — nessun candidato potrà più migliorare su un riferimento regredendo in silenzio sull'altro. A2 fallirebbe anche questo.
- **G4 forma**: Pearson ≥ baseline S0D − 0,02 per giunto (baseline **[0,9175, 0,7794]** vs IK), rapporto di ampiezza in **[0,80, 1,25]**; `sign_agreement` **VOID** se il riferimento è degenere, con lo stato di degenerazione sempre registrato.
- **G5 escursione negativa della caviglia**: min della caviglia realizzata nella finestra [0,55, 0,80] ≤ **−0,03**, **vincolante solo se S1C-0 restituisce REACHABLE**; altrimenti resta **BLOCKER APERTO** e non può far passare né bocciare alcun candidato.
- Nessuna metrica offline dichiara walking; nessun gate è rilassabile una volta congelato.

**Verifica numerica del rifiuto dell'architetto:** applicando i gate S1C alle metriche reali di A2, **G2 e G3 falliscono entrambi** e `all_binding_pass = False`. Il tuo rifiuto della promozione qualitativa è quindi riprodotto numericamente dal protocollo, non solo condiviso.

## 4. Test statici
`test_v26b_s1c_protocol.py`: **PASS, 53 check**. Copertura: pin/tamper rev3v e guardia sul lettore dell'emendamento; 4 token negativi più le tre guardie di stadio futuro (probe/fit/rollout), ciascuna che nomina il proprio token; assenza di primitive di fit e closed-loop nel modulo; **convenzione di decodifica** verificata sui valori estremi (a = ±1 → q_knee ∈ {0, −1,5}, q_ankle ∈ {+0,7, −0,7}); **il bug del sign-agreement** riprodotto e corretto su dati sintetici (riferimento degenere → VOID, non 1,0); la degenerazione del riferimento sano e la presenza del tratto negativo nel target IK misurate sui dati reali; l'audit della catena comando→reference→giunto con le sue cinque evidenze; **ciascun gate vincolante che fallisce isolatamente**; **A2 che fallisce G2 e G3** sui numeri reali; G5 non vincolante finché non è provata la raggiungibilità, e vincolante-e-fallita quando il minimo non è negativo; griglia finita a 3 candidati con `open_search: false`; σ irrisolta; no-clobber dell'artefatto di audit.

Trasparenza: un mio check confrontava una stringa con maiuscola diversa da quella congelata (`ONLY` vs `only`) — reso case-insensitive prima della consegna. Nessuna soglia toccata.

## 5. Artefatti
| Artefatto | SHA-256 |
|---|---|
| `…/v26b_bridge_2026-08-24/v26b_amendment_rev3v_s1c_protocol.json` | `889f1068a9ddb8db8241b62b569a85fbc6baf3d7673ae626c4df5e51902ed984` |
| `…/v26b_bridge_2026-08-24/v26b_s1c_protocol.py` | `0a3987d8f7630c28917ba4a062d56dc2657ce60e65092691522f28822742b53c` |
| `…/v26b_bridge_2026-08-24/test_v26b_s1c_protocol.py` | `aed01c03f73940f11e095e0491bdc5cf9d0909c064118619ccf55a7956d649b0` |
| `…/2026-08-24_V26B_anchors_r1/s1c_protocol/v26b_s1c_audit_20260824_204349.json` | `98d69eadd2ca7e91ad389ae12ec623b494a13df06137043072263c0b40990b76` |

## 6. Stato e decisione richiesta
**STOP per la tua revisione.** Nessun fit, rollout, probe o promozione eseguito; A2 resta NON-DEPLOYABLE e quarantinati restano A1, A3–A6; σ irrisolta.

Due decisioni sono tue: **(a)** autorizzare o no il probe di fattibilità S1C-0; **(b)** stabilire se il requisito di caviglia negativa debba diventare una **questione di configurazione di produzione** (reference governor / slew / SEA, oppure applicabilità del criterio B3 a questa morfologia) invece di un obiettivo di apprendimento. La mia raccomandazione, sui dati: **(b) prima di (a)** — perché la prova che una policy comanda già −0,49 senza ottenere nulla al giunto rende improbabile che un qualunque riaddestramento cambi l'esito, e S1C-1 rischierebbe di spendere tre fit contro un obiettivo che il percorso di reference non può consegnare.
