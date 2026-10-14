# Protocollo di verifica dei reset da pose protesiche casuali

Data: 13 ottobre 2026. Instruction check token: CMC_AGENT_OK_2026.
Autorizzazione: verifica richiesta dall'utente; scope confermato knee e ankle
protesici, con il resto biologico inizializzato dall'IK; collaborazione Claude
come nella precedente indagine, Opus 5.5 effort massimo.

## Domanda e banco

Misurare se V26C best47 recupera una sequenza di gait dopo un reset da una posa
protesica casuale ammessa dal modello. Checkpoint congelato AB06, actor35/critic84.
Tempo iniziale 13.946870983805102 s e riferimento biologico/GRF destra fissi.
Cinque secondi richiesti per caso, inferenza deterministica seed123.
Nessun aggiornamento policy/critic, nessuna modifica di plugin, controller,
profili, limiti, modello o sorgenti operativi.

## Campionamento prima degli esiti

Generatore Python stdlib random.Random(20261013), 400 coppie IID uniformi:
q_knee in [-1.20,-0.03] rad e q_ankle in [-0.55,+0.55] rad.
Congelare pool completo, ordine, versioni e SHA prima della selezione.
Screening di tutte le proposte senza integrazione o inferenza policy:
coordinate entro ROM del modello, FK/stati finiti, penetrazione delle sfere
primarie LEFT <=27 mm e norma della forza primaria LEFT <=5 BW
(BW=mass*9.80665 N), come limite force ordinario. La guardia dinamica di
penetrazione 28 mm non cambia. Il millimetro di margine iniziale è parte
del dominio ammesso, non una nuova soglia del runtime.

Scegliere i primi20 candidati ammissibili in ordine. Conservare tutte le400
decisioni, escluse incluse, insieme a quota/punti/FSM/carico iniziali.
AIR ammesso senza ulteriore cap di clearance. Ammissibilità significa soltanto
ROM e geometria/forza del modello: equilibrio statico e self collision non
vengono certificati da questo gate. Il piano primario e il piano sensor-only
del detector non vanno confusi. Nessuna sostituzione dei failure dinamici.
Se il pool non offre20ammessi, riportare l'insufficienza senza cambiare seed
o dominio dopo aver osservato esiti.

## Bootstrap e controlli

Per ogni posa random:
- velocità dei due giunti zero; tutte le coordinate/velocità biologiche IK;
- motor_angle=q e motor_speed=0, deflessione iniziale zero, integrali PI zero;
- endpoint precedente e reference servita q=random, v=a=0;
- stato interno governor C2 q=random, v=a=0;
- prime del detector/FSM dopo la posa effettiva, senza HS/TO sintetici.

L'adapter vive solo nel harness. La reference protesica resta sul bootstrap
finché arriva il primo segmento della policy, poi usa il governor originale.
Nessuna query può reintrodurre la IK protesica: out-of-segment non previsto
è un errore tecnico, non un fallback. La biologia continua a usare IK.
L'osservazione di carico iniziale conserva il contratto originale zero fino
alla prima sample; il gate usa invece il contatto fisico effettivo.

Controllo principale: posa nominale IK con qdot protesiche zero e governor
(q_nom,0,0), identico ai random per tutti i fattori salvo posizione iniziale.
Controllo storico legacy: riuso dei rollout IK originali del 12 ottobre,
già identici byte per byte ai replay precedenti; gli ingressi protetti vengono
riverificati. Non è prevista una nuova esecuzione legacy nella campagna odierna.
Il controllo nominale e gli artefatti storici sono esclusi dal denominatore N20.

Prima dei rollout: test offline del wrapper e una prova bootstrap/first-segment
senza integrazione, verifica actual/reference/SEA/endpoint/FSM e conservazione
esatta della biologia. Sigillare manifest finale solo dopo review sorgenti.
Ogni errore locale è registrato; il batch continua con i casi selezionati.
Hash drift globale interrompe la campagna senza mutare gli ingressi.

## Esiti e metriche

Esito primario di recupero gait nel banco:
- 500 step / 5 s con normale episode_time_limit;
- almeno2 cicli actor validi e primo HS accettato entro3s;
- zero invalid/rejected/degraded/timeout/resync e zero HS cancellati;
- guardie fisiche ordinarie rispettate, penetrazione1ms<28mm;
- RMSE actual-served per giunto <=2° dopo0.5s;
- almeno due cicli completi con, in ciascuno, escursione knee>=0.60rad
  e ankle>=0.30rad; il range del solo transitorio iniziale non conta.

Riportare separatamente completamento sicuro, acquisizione primoHS/TO/ciclo,
numero di cicli, forma/escursione, carico/impulso, reserve, clipping policy,
comando SEA e clamp motore. Le ultime metriche sono diagnostiche, senza
nuovi hard gate ad hoc. Return è descrittivo: cambia anche l'accredito
startup/FSM e non dimostra da solo recupero meccanico o migliore apprendimento.

Nessun reward/evento viene ricostruito per un segmento non restituito.
Failure fisici fanno parte del denominatore. Errori tecnici/postprocessing
sono distinti: non trasformano un rollout finito in un failure scientifico
e non vengono eliminati dal conteggio senza esplicitare l'incertezza.
Per ciascun rollout con dati, sette plot canonici sotto
root/plot/random_pose_reset_2026_10_13/<case>/, inclusi casi interrotti.
Se non ci sono sample sufficienti, riportare l'indisponibilità delle figure.

## Statistica e limiti

Stima e intervalli Clopper-Pearson95% condizionati al dominio ammesso.
N indipendente è il numero di pose estratte, non step/cicli/sample.
Anche20/20 non prova capacità universale: lower bound one-sided95%≈86.1%.
Con errori tecnici irrisolti, riportare bound pessimista/ottimista sulNselezionato.
Frazioni accettate sulpool400 e distribuzione dei rifiuti sono visibili.

Questa prova riguarda una rete, AB06, un timing biologico, qdot0 e il dominio
dichiarato. Non è reset di tutto il modello, avvio statico da fermo, prova
senza supporto biologico, effetto causale sul training o qualifica hardware.
Le esecuzioni native attestano macOS arm64; il harness deve restare portabile
a Windows con interprete e plugin corrispondenti.

## Collaborazione indipendente

Audit Codex delle sorgenti e piano statistico prodotti senza leggere Claude
o i risultati nuovi. Claude analizza indipendentemente sorgenti/progetto;
Codex sigilla le conclusioni prima di leggere la risposta esterna.
Dopo la campagna Claude riceve i due audit e una capsula numerica compatta
per confronto e critica. Modello effettivo, effort richiesto, comandi, input
e output conservati. Nessun fallback di modello.

