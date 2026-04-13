Sto lavorando su un simulatore OpenSim/Python che vuole riprodurre il Computed Muscle Control (CMC) il più fedelmente possibile sul lato biologico, mantenendo però due SEA protesici controllati da controllori high-level nel simulatore.

Voglio che tu modifichi il codice del repository per raggiungere questi obiettivi:

OBIETTIVO PRINCIPALE
- Il tracking biologico deve essere prodotto principalmente dai muscoli.
- I reserve actuators devono rimanere nel modello solo come supporto residuale, da usare quando il problema muscolare non è perfettamente fattibile, ma NON devono essere il principale attuatore del tracking.
- Il comportamento deve essere il più possibile CMC-like:
  1) outer loop di tracking cinematico -> qddot_des sui DoF biologici
  2) inverse dynamics -> tau_bio richiesto
  3) static optimization / muscle recruitment -> attivazioni muscolari che producono tau_bio
  4) reserve actuators usati solo come residual / supporto numerico
- In parallelo, i SEA devono restare controllati da controllori high-level nel simulatore Python, mentre il plugin C++ rimane il controllore low-level di coppia.

VINCOLI IMPORTANTI DA RISPETTARE
1. NON semplificare il problema tornando a far fare il tracking biologico alle reserve.
2. NON cambiare l’architettura high-level dei SEA.
3. NON assumere che l’ordine del CoordinateSet coincida con l’ordine delle mobilità Simbody: il mapping coord_name -> mobility_index già inferito in model_loader.py è corretto e va preservato.
4. NON rompere i bugfix di stabilità già presenti:
   - baseline ID con tutti i controlli zero
   - azzeramento della deflessione SEA nel baseline (`motor_angle = theta_joint`)
   - aggiornamento di `motor_angle` / `motor_speed` nel runner per il non-impedance mode
   - bypass della dinamica senza `realizeAcceleration()`
5. Mantieni la compatibilità con l’attuale loop forward in Python.

ARCHITETTURA SEA DA PRESERVARE
Il plugin SEA C++ implementa il low-level controller.
Il simulatore Python deve continuare a fare solo il controllo high-level.
Devi trattare il plugin come segue:

- Il simulatore calcola un comando high-level normalizzato `u_ref`.
- Il plugin SEA usa internamente `tau_ref = u_ref * F_opt`.
- In non-impedance mode, il plugin non applica direttamente `u_ref * F_opt` al giunto, ma insegue quella coppia tramite la dinamica del motore e della molla.
- Quindi il simulatore deve continuare a:
  - calcolare il controllo high-level SEA in unità fisiche di coppia:
    `tau_cmd = tau_ff + Kp*(q_ref - q) + Kd*(qdot_ref - qdot)`
  - normalizzare solo alla fine:
    `u_ref = clip(tau_cmd / F_opt, -1, +1)`
  - aggiornare `motor_angle` in modo coerente nel runner quando necessario per il non-impedance mode.
- NON spostare il tracking high-level dei SEA nel plugin.
- NON far sì che i SEA partecipino al tracking biologico o alla SO biologica.

PROBLEMA ATTUALE DA RISOLVERE
Al momento il codice ha disattivato il vero contributo muscolare nella SO perché il mapping attuale usa una stima lineare del tipo:
`tau_muscle ~= moment_arm * activation * Fmax`
che non è coerente con la forza muscolare istantanea OpenSim/Thelen. Il risultato è che:
- `use_muscles_in_so=False`
- il blocco muscolare della matrice A è nullo
- tutta la coppia biologica richiesta viene applicata tramite reserve actuators

Questo va cambiato.

COSA VOGLIO CHE TU IMPLEMENTI
Voglio una versione migliorata della static optimization biologica che usi i muscoli in modo fisicamente più coerente con OpenSim, evitando che le reserve dominino il tracking.

Proponi e implementa una soluzione robusta, scegliendo una di queste due strategie (preferisci la migliore praticabile nel codice attuale):

STRATEGIA PREFERITA
A. Linearizzazione locale della produzione di coppia muscolare rispetto all’attivazione
- Per ogni frame, costruisci una mappa muscoli -> coppie biologiche che sia coerente con lo stato corrente del modello, non solo con `Fmax`.
- Usa i muscoli reali OpenSim nello stato corrente per stimare, per ciascun muscolo j:
  - la forza muscolare istantanea o il guadagno locale di forza rispetto all’attivazione
  - il relativo contributo alle coordinate biologiche tramite i moment arm
- In pratica, il blocco muscolare della SO non deve più essere:
  `R[i,j] * Fmax[j]`
  ma qualcosa di più coerente con la forza muscolare istantanea del modello reale, ad esempio una linearizzazione locale del torque-per-activation attorno allo stato corrente.
- La SO deve usare questa matrice aggiornata frame-by-frame per distribuire `tau_bio` soprattutto ai muscoli.

STRATEGIA ALTERNATIVA
B. Static optimization residualizzata
- Se una linearizzazione locale completa è troppo fragile, implementa una SO in cui:
  - i muscoli producono la parte dominante della coppia biologica
  - le reserve chiudono solo il residuo
- In questo caso devi comunque migliorare il blocco muscolare oltre al semplice `moment_arm * Fmax`, oppure introdurre un solve iterativo / correttivo che riduca il mismatch fra coppia muscolare prevista e coppia effettiva nel modello OpenSim.

COME DEVI USARE LE RESERVE
Le reserve devono diventare residual actuators, non primary actuators.
Quindi:
- alza molto la penalizzazione delle reserve nel costo
- restringi i bounds se possibile senza compromettere la fattibilità numerica
- usa fallback numerici robusti, ma non permettere alla soluzione di “scaricare” sistematicamente il tracking sulle reserve
- aggiungi logging diagnostico per mostrare:
  - quota di tau_bio coperta dai muscoli
  - quota coperta dalle reserve
  - norma dei controlli reserve
  - eventuale infeasibility / mismatch

FILE SU CUI LAVORARE
Concentrati soprattutto su:
- config.py
- static_optimization.py
- simulation_runner.py
- inverse_dynamics.py
- eventualmente model_loader.py se servono cache aggiuntive dei muscoli o degli state variable index
- non modificare inutilmente il plugin C++ SEA, salvo chiarimenti minimi nei commenti o piccole fix non funzionali

RICHIESTE SPECIFICHE DI IMPLEMENTAZIONE
1. In config.py:
   - abilita una modalità muscle-driven vera per il tracking biologico
   - aggiungi parametri chiari per:
     - reserve weight
     - reserve bounds “residual mode”
     - eventuale scelta della strategia di muscle mapping
     - eventuali pesi di regolarizzazione sulle attivazioni
   - aggiorna i commenti per spiegare la nuova logica

2. In static_optimization.py:
   - sostituisci l’attuale blocco muscolare basato solo su `moment_arm * Fmax`
   - implementa una mappa muscolo->torque più coerente con il modello muscolare reale nello stato corrente
   - mantieni warm-start, solver SLSQP/OSQP, fallback robusto
   - aggiungi un report diagnostico opzionale per frame
   - fai in modo che la soluzione privilegi sempre i muscoli e usi le reserve solo come residual

3. In simulation_runner.py:
   - mantieni invariata la logica generale del loop
   - mantieni invariato il controllo high-level dei SEA
   - aggiungi eventualmente logging delle grandezze biologiche:
     - somma attivazioni
     - norm reserve controls
     - torque tracking error residuo
   - non rompere `_update_sea_motor_state()`

4. In inverse_dynamics.py:
   - mantieni il bypass attuale e il baseline zero-actuator
   - mantieni l’azzeramento delle molle SEA nel baseline
   - non reintrodurre dipendenze da `realizeAcceleration()`

5. Nei commenti/docstring:
   - documenta chiaramente che il plugin SEA è un low-level torque controller:
     il simulatore calcola `u_ref`, il plugin insegue `tau_ref = u_ref * F_opt`
   - documenta chiaramente che il lato biologico deve essere il più possibile CMC-like e muscle-driven

CRITERI DI SUCCESSO
La modifica è accettabile solo se ottengo queste proprietà:

A. Biologico
- le attivazioni muscolari non sono più quasi tutte zero
- le reserve non sono più il canale principale del tracking biologico
- la maggior parte della coppia biologica è coperta dai muscoli
- il tracking resta stabile e non diverge nei primi step

B. Protesico
- i SEA continuano a seguire la traiettoria con il controllore high-level nel simulatore
- il plugin continua a fungere da low-level torque controller
- non viene alterata la semantica `tau_ref = u_ref * F_opt`

C. Numerico
- niente `nan/inf`
- niente rottura dei bugfix su mobility indices, baseline ID o stato motore SEA
- la simulazione completa deve restare eseguibile con i default o con minime regolazioni dei parametri

OUTPUT CHE VOGLIO DA TE
1. Modifica direttamente i file necessari.
2. Spiega in modo sintetico la strategia scelta.
3. Elenca i tradeoff della soluzione implementata.
4. Indica quali parametri di config conviene ritoccare per il tuning.
5. Se durante l’implementazione trovi un limite strutturale che impedisce una replica fedele del CMC, non aggirarlo in modo nascosto:
   - spiegalo esplicitamente
   - implementa la migliore approssimazione robusta possibile
   - mantieni comunque il principio: muscoli principali, reserve residuali, SEA controllati high-level dal simulatore.

Lavora in modo incrementale ma consegna codice funzionante, coerente e ben commentato.