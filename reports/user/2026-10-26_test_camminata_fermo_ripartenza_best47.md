# 2026-10-26 — Best47: camminata, fermo e ripartenza

**Esito: la struttura attuale completa la sequenza temporale, ma non esegue il fermo richiesto.** Durante i tre secondi con riferimento biologico e superficie fermi, la protesi continua a muoversi e completa un ciclo HS–TO–HS valido. La successiva camminata viene raggiunta, ma non dimostra una ripartenza da stazione eretta, perché l'arresto non è avvenuto.

Approfondimento del 2026-10-26: la [diagnosi successiva](2026-10-26_diagnosi_fermo_ripartenza_best47.md) distingue policy, FSM e limiti del protocollo. La prova verifica una perturbazione sintetica con riferimenti fermi; non impartisce un comando esplicito di arresto e non parte da un appoggio bilaterale simulato verificato.

Il problema verificato è la capacità della catena rete → policy → generatore di riferimenti → controllore SEA → dinamica OpenSim → detector geometrico → FSM binaria V26 → adapter → FSM actor v3 di gestire camminata → fermo → camminata. È stato usato il checkpoint congelato `MLP_V26C_J20_sigma0p010_morph_equal5p0_ppo50_2026-09-01/rl_module_best`, con preset ex novo corrente, inferenza deterministica e seed 123. Nessun training o adattamento dei pesi.

La strategia è una perturbazione sintetica semplice del caso AB06 treadmill. Una mappa temporale regolare modifica la velocità di avanzamento della cinematica biologica; velocità e accelerazioni sono calcolate con la regola della catena. Le GRF prescritte destre, COP e momenti seguono la stessa mappa, conservando le ampiezze. La velocità della superficie di contatto online viene scalata agli stadi RK4, nella direzione originale. I giunti protesici restano comandati dalla policy. I fattori sono relativi alla superficie calibrata: non rappresentano velocità sperimentali validate in m/s.

La pausa è stata scelta prima di osservare la policy, in un intervallo di doppio appoggio dei dati sorgente con entrambe le forze verticali ≥100 N; questo non impone doppio appoggio alla protesi simulata. Il protocollo definitivo dura 8,31 s, dall'istante assoluto 13,946871 s:

| Fase | Tempo dall'avvio | Fattore |
|---|---:|---:|
| Camminata | 0–2,307 s | 1 |
| Rallentamento | 2,307–2,807 s | 1 → 0 |
| Fermo richiesto | 2,807–5,807 s | 0 |
| Accelerazione | 5,807–6,307 s | 0 → 1 |
| Camminata finale | 6,307–8,310 s | 1 |

Il controllo a velocità costante completa 9,50 s e 5 cicli validi. La prova completa tutti gli 831 step da 10 ms, con 4 cicli validi, nessun evento rifiutato, nessuna risincronizzazione e nessun timeout. Entrambe terminano normalmente per `episode_time_limit`, senza clipping delle azioni. Prima della perturbazione, su 230 campioni comuni, la differenza massima di posizione articolare è 1,17×10⁻⁵ rad; quella delle azioni è 6,18×10⁻⁵.

La sopravvivenza numerica non equivale all'arresto. Nella pausa, i riferimenti biologici hanno velocità e accelerazione nulle, ma le escursioni effettive protesiche sono:

| Giunto | Pausa completa | Pausa esclusi i primi 0,5 s | Velocità RMS nell'ultima finestra |
|---|---:|---:|---:|
| Ginocchio | 0,69880 rad / 40,04° | 0,66046 rad / 37,84° | 0,9123 rad/s |
| Caviglia | 0,51803 rad / 29,68° | 0,51803 rad / 29,68° | 0,6725 rad/s |

Sono misure dalla trace a 10 ms. Anche i riferimenti serviti continuano a oscillare. Un intero ciclo valido, **3,409–5,625 s**, cade dentro la pausa: durata 2,216 s con fattore sempre zero. Nella camminata finale compare un ciclo interamente contenuto, **6,734–8,228 s**, di 1,494 s. La penetrazione massima è 24,873 mm, sotto la guardia di episodio a 28 mm; il minimo del bacino è 0,95242 m. La metrica aggregata `reserve_norm_nm` sale a 778,68, contro 529,18 del controllo. Non è una coppia fisica omogenea: aggrega anche reserve traslazionali in N, quindi la precedente etichetta N·m era impropria. Il sostegno biologico/residuale resta parte del risultato; la diagnosi successiva riporta anche le componenti con unità distinte.

La sonda separata `fsm_probe.py`, senza rete né dinamica, conferma un limite distinto: con contatto stabile la FSM entra in `TIMEOUT` dopo 2,209 s dall'ultimo HS, superando il limite stance di 2,20 s. Nel rollout completo il movimento prosegue e rinnova gli eventi. Il contratto actor non contiene un comando esplicito di arresto/velocità desiderata e la FSM non ha uno stato dedicato alla stazione eretta. Non sono stati disabilitati i timeout per ottenere il risultato.

Sono stati aggiunti soltanto strumenti sperimentali sotto `Trajectory Generator/validation/speed_transitions_2026_10_26/`: `run_transition.py`, `fsm_probe.py`, `analyze_results.py` e `render_canonical.py`, con relativi artefatti, questo report e grafici nella root `plot/`. Verificati derivazione cinematica, aggiornamento nativo delle proprietà di contatto, copertura temporale, 17 STO e stream diagnostici finiti. Gli hash di 41 input protetti confermano invariati checkpoint, sorgenti operativi, modelli, preset e plugin.

Il primo tentativo `stop_restart` è escluso: il suo ultimo segmento frazionario, a 8,307129 s, non rispettava la griglia del detector. La sola durata dell'harness è stata allineata a 8,31 s e la prova ripetuta integralmente. L'errore non è attribuito alla policy.

Esecuzione su macOS arm64, CPU, Python 3.10.20 e OpenSim 4.5.2; nessuna esecuzione nativa Windows. È un singolo stress test sintetico, senza dati misurati di arresto/ripartenza: non certifica equilibrio autonomo né generalizzazione.

Evidenze: [rollout definitivo](<../../Trajectory Generator/validation/speed_transitions_2026_10_26/stop_restart_aligned/>), [analisi](<../../Trajectory Generator/validation/speed_transitions_2026_10_26/analysis.json>), [sonda FSM](<../../Trajectory Generator/validation/speed_transitions_2026_10_26/fsm_probe.json>), [sette plot canonici](../../plot/speed_transitions_2026_10_26/stop_restart_aligned/). I grafici di ciclo usano cicli effettivamente validati; gli overlay healthy sono diagnostici.

Per ripetere dalla radice, con `envCMC-rllib` attivo, senza sovrascrivere i risultati:

```sh
python "Trajectory Generator/validation/speed_transitions_2026_10_26/run_transition.py" --case stop_restart --suffix _repeat
```

**TODO aperto:** definire e verificare il comportamento di arresto/stazione eretta/ripartenza, inclusi riconoscimento dell'intento e contratto FSM. Nessuna correzione funzionale è stata implementata in questa prova.
