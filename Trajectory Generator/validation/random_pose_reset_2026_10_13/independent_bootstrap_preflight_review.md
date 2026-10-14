# Review indipendente del preflight e bootstrap — 2026-10-13

Confermati in sola lettura 401 reset statici: controllo coerente + 400 candidati, **364 ammissibili e 36 respinti**. La selezione coincide con i primi 20 ammissibili nell'ordine congelato; nessun risultato di rollout dinamico è stato letto.

Penetrazione ricalcolata dai punti nativi delle otto sfere sinistre rispetto al piano primario: errore massimo 3.47e-18 m. Somma delle forze e norma confrontate col guard 5 BW ricavato dalla massa XML 78.09885033000 kg (3829.440452943 N). Gate 27 mm verificato; nessun cap di clearance in aria.

Fra gli ammissibili, 211 reset (controllo incluso) hanno tutte le sfere primarie in aria; 54 hanno contemporaneamente almeno un point-detector attivo. I profili usano piani diversi: primario y=-0.009503955 m, detector y=0.014820823 m, oltre a punti diversi. È una distinzione di contratto sensoriale, non un heel strike valido inventato.

Confermati i quattro probe nativi senza integrazione:

| Caso | Pose q/ref/SEA/endpoint | v/a/PI iniziali | Primo segmento | Stato FSM | HS/TO/cicli |
|---|---|---|---|---|---|
| coherent_nominal | coerenti | zero | q iniziale coerente, v=a=0 | STANCE_AFTER_HS | 0/0/0 |
| random_003 | coerenti | zero | q iniziale coerente, v=a=0 | STANCE_AFTER_HS | 0/0/0 |
| random_006 | coerenti | zero | q iniziale coerente, v=a=0 | STANCE_AFTER_HS | 0/0/0 |
| random_025 | coerenti | zero | q iniziale coerente, v=a=0 | WAIT_HS | 0/0/0 |

Gli ingressi actor per q e previous endpoint coincidono con la pose dopo conversione float32; velocità, accelerazioni della reference e comando SEA sono zero. Gli anchor e lo stato interno del governor coincidono con le condizioni iniziali, senza ritorno alla IK protesica nella prima pianificazione. La biologia q/qdot coincide con la IK nominale del controllo precedente; raw action viene trasformata in (1,2) e clippata con la stessa regola del wrapper originale.

Nessun verdetto di recupero, cammino autonomo o qualificazione scientifica. L'analisi usa punti FK già registrati; non ricalcola una nuova FK OpenSim. Tutti gli input letti e gli SHA sono nel JSON e risultano invariati. Nessun file congelato o output nativo è stato modificato.

Controllo indipendente aggiuntivo: 76 file congelati verificati con SHA-256; nessuna differenza: True.
