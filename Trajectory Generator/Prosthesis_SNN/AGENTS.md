# Istruzioni per gli agenti - Prosthesis_SNN

Questa repository contiene il core SNN per generare traiettorie cinematiche
destinate a protesi SEA in un modello muscoloscheletrico (.osim).

L'obiettivo della rete è generare traiettorie cinematiche da fornire come
segnale di riferimento al controllore dei SEA.

Il contesto della simulazione è documentato in
[docs/SIMULATOR_CONTEXT.md](docs/SIMULATOR_CONTEXT.md).

## TL;DR

- **Non toccare** il plugin C++ SEA né la semantica del controllore CMC-like
  senza richiesta esplicita: in caso di dubbio, fermarsi e chiedere conferma.
- **Interfaccia attesa** verso il simulatore:
  `q_ref, qdot_ref, qddot_ref = provider.get(t, state=None)`.
- **Perimetro rete**: tutto il lavoro relativo a rete neurale, policy,
  training e generatori appresi vive in `Trajectory Generator/`, non nel root
  del simulatore CMC-like.
- **Piattaforma primaria**: Windows x86_64. Portabilità verso macOS arm64 e
  altre piattaforme è **deferred** e non vincola il design corrente.
- **Libreria-policy**: lato training si possono usare tutte le librerie
  necessarie (skrl, OpenSim, Hydra/OmegaConf, ecc.).
- **Decisioni aperte**: vivono in
  [docs/TODO_integration.md](docs/TODO_integration.md), mai come assunzioni
  nascoste nel codice.
- **Workflow report**: comandi `start_day` / `create_report` / `end_day` —
  vedi sezione [Comandi custom](#comandi-custom).
- `CLAUDE.md` ha precedenza solo per Claude Code; per tutti gli altri agenti
  questo file è la fonte di verità.

## Vincoli fondamentali

- L'inferenza a runtime deve funzionare su Windows x86_64. La portabilità
  verso macOS arm64 è deferred: il codice non deve sacrificare nulla per
  garantirla, ma evitare scelte gratuitamente OS-specifiche resta buona
  pratica per un eventuale recupero futuro.
- Usare PyTorch e snntorch per il core SNN. Il core (`prosthesis_snn/`) deve
  mantenere un percorso CPU funzionante; CUDA è opzionale e deve avere
  fallback automatico a CPU. La pipeline di training può dipendere da
  qualsiasi libreria utile (skrl, OpenSim, ecc.) senza vincoli di import dal
  core.
- `CMC-like-Simulator - Claude` è la cartella del simulatore. 
- Non modificare la semantica del driver low-level SEA (plugin C++) o del loop
  di controllo CMC-like. Se una task sembra richiedere queste modifiche,
  **fermarsi e chiedere conferma esplicita**.

## Interfaccia attesa

L'integrazione futura con il simulatore deve passare da un provider di
riferimenti, non dalla riscrittura diretta del controllore.

Il contratto del provider è:

```python
q_ref, qdot_ref, qddot_ref = provider.get(t, state=None)
```

Questo rispecchia il contratto corrente di `KinematicsInterpolator.get(t)`
usato dal simulatore CMC-like.

Il primo target SNN è solo protesico:

- `pros_knee_angle`
- `pros_ankle_angle`

Le coordinate biologiche devono continuare a essere fornite dal percorso IK /
spline esistente finché l'utente non espande esplicitamente lo scope.

## File da preservare

- `prosthesis_snn/model.py`: modello SNN portabile e gestione della memoria.
- `prosthesis_snn/reference_provider.py`: contratto adapter verso il
  simulatore.
- `prosthesis_snn/config.py`: dataclass di configurazione del core
  (inference-time). La pipeline di training può avere una config separata
  (anche basata su Hydra/OmegaConf) senza propagarla nel core.
- `docs/TODO_integration.md`: decisioni aperte e assunzioni.
- `docs/EXTRACTION_NOTES.md`: parti riusate o intenzionalmente escluse dal
  repository sorgente.

## Workflow di modifica

- Prima di modificare codice, leggere i file coinvolti e preservare le
  invarianti già presenti, in particolare la separazione high-level Python /
  low-level plugin C++ dei SEA.
- Una task si considera completata solo se il provider/modello resta
  verificabile in isolamento, senza simulatore OpenSim caricato.

## Verifiche attese

Prima di consegnare modifiche:

- Eseguire `tests/smoke_test.py` quando `torch` e `snntorch` sono installati.
- Verificare prima l'inferenza su CPU (Windows x86_64).
- Se si usa CUDA, verificare che il fallback su CPU continui a funzionare.
- Per modifiche ai provider, verificare che il provider ibrido preservi i
  riferimenti biologici e sovrascriva solo i valori di ginocchio/caviglia
  protesici.

## Policy per i TODO

Se un dettaglio implementativo non è ancora stato deciso dall'utente, non
cristallizzarlo nel codice come assunzione nascosta.

Aggiungerlo invece a `docs/TODO_integration.md` e usare un default piccolo,
esplicito e facile da sostituire.

## Comandi custom

| Comando         | Azione                                                                                                                                | Output                                                  |
|-----------------|---------------------------------------------------------------------------------------------------------------------------------------|---------------------------------------------------------|
| `start_day`     | Leggere `AGENTS.md` e l'**ultimo** daily report in `reports/daily/`; propagare i TODO ancora aperti.                                  | (nessun file)                                           |
| `create_report` | Scrivere un report utente sulla task corrente. **Solo su comando esplicito**, mai in autonomia.                                       | `reports/user/[YYYY-MM-DD]_[titolo_significativo].md`   |
| `end_day`       | Aggregare i report utente del giorno in un daily report. Se non esistono report utente, scrivere comunque un daily breve con i TODO ereditati. | `reports/daily/[YYYY-MM-DD]_daily-report.md`            |

### Convenzioni report

- **Formato data**: ISO `YYYY-MM-DD` (es. `2026-04-14`).
- **Lingua dei report**: italiano.
- **Contenuto report utente**: almeno _problema, soluzione, strategia, file
  modificati, test/verifiche eseguite_.
- **Propagazione TODO**: i TODO aperti nei report utente vanno riportati nel
  daily report corrispondente e propagati nei daily successivi finché non
  sono risolti.

### Esempi di titoli

- Report utente: `2026-04-14_recruitment_muscle_driven.md`
- Daily report: `2026-04-14_daily-report.md`
