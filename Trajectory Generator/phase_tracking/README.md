# Gestione delle fasi del passo

Componenti condivisi dall'ambiente di generazione delle traiettorie, indipendenti
dalla specifica rete in `baseline_MLP/`.

| Modulo | Ruolo |
|---|---|
| [`binary_phase.py`](binary_phase.py) | Interpreta i campioni binari tallone/punta, applica il debounce e trasferisce gli eventi HS/TO alla FSM dell'actor. Mantiene distinte le classi base e le sottoclassi V26, che richiedono contatto stabile del tallone per iniziare una stance. |
| [`prosthetic_phase_fsm.py`](prosthetic_phase_fsm.py) | Mantiene il ciclo del passo per l'actor, verifica sequenza e tempi degli eventi e gestisce timeout e recupero della sincronizzazione. |

Il detector geometrico del simulatore e in
[`binary_phase_detector.py`](../../online_GRF/binary_detector/binary_phase_detector.py),
insieme alla geometria operativa in
[`online_GRF/binary_detector/`](../../online_GRF/binary_detector/).
Legge lo stato OpenSim e produce i campioni tallone/punta; le FSM di questa
cartella ne interpretano gli eventi per il generatore di traiettorie.
Il codice del simulatore è nel core parallelo
[`CMC-like Simulator/`](<../../CMC-like Simulator/README.md>); geometria e
runtime GRF restano risorse condivise nella radice del workspace.

Dal 2026-10-20 i due moduli sono riuniti qui senza modificarne il contenuto.
Prima degli import condivisi chiamare `_bootstrap.ensure_sim_paths()` dal
bootstrap della baseline: aggiunge questa cartella usando solo la libreria
standard. Gli import canonici restano `binary_phase` e
`prosthetic_phase_fsm`, come nei pickle e nei worker Ray.

Per caricare manualmente pickle che citano i precedenti `binary_phase_fsm`,
`binary_phase_fsm_v26`, `binary_phase_adapter` o `binary_phase_adapter_v26`,
importare prima `binary_phase` dopo il bootstrap: il modulo registra gli alias
in memoria. I quattro vecchi file erano stati riuniti in `binary_phase.py`;
le classi e i contratti restano distinti.

Gli inventari e gli hash congelati descrivono il layout storico e conservano
i percorsi originali. La verifica di quei record richiede il checkout
originale completo; non certifica il nuovo layout.
