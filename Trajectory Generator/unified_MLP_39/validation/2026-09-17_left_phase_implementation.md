# 39D: teacher sulla fase sinistra e correzione worker

Instruction check token: CMC_AGENT_OK_2026

## Esito

Implementati il metronomo prescribed sinistro senza shift e il fix dei worker.
Verifica macOS arm64 completata: **61 test distinti superati**, prova OpenSim
visibile/mascherata sui tre start e **due aggiornamenti PPO da 192 transizioni**,
separati da salvataggio e ripresa completa del checkpoint. Nessun training lungo
o confronto statistico con V26 è stato avviato.

## Teacher e osservazioni

`phase_target.py` usa gli heel strike prescribed **sinistri**, le coordinate
`pros_knee_angle` e `pros_ankle_angle` e shift **zero**. Riutilizza l'algoritmo
V26 di ricampionamento in fase, media dei cicli e spline cubica periodica.
La velocità è la derivata della spline rispetto alla fase divisa per il periodo
prescribed locale. I dettagli di selezione dei cicli ed estrapolazione del clock
sono documentati nel [README](../README.md).

`env.py` costruisce il teacher prima di calcolare le scale delle loss. Reward,
quattro feature e diagnostica usano lo stesso target. La maschera azzera soltanto
i quattro ingressi; il teacher imitativo resta invariato. Il clock prescribed
dedicato non alimenta direttamente l'actor: i suoi due slot storici rimangono
zero, e il suffisso privilegiato non riceve nuove copie dei target.

Dimensioni conservate: **39 actor / 88 complessive**. Normalizzazione degli
ingressi conservata: posizioni /1 e velocità /10. Le scale delle loss restano
calcolate dalle escursioni del teacher, ora periodico. Con gli asset correnti
il profilo usa cinque cicli e periodo medio 1.575026 s; escursioni di posizione
ginocchio/caviglia 0.788872/0.499580 rad, identiche sui tre start.

Il contratto è ora `unified_mlp39_v2`: i checkpoint v1, basati sul lookup temporale,
vengono rifiutati. Checkpoint e artefatti storici non sono stati riscritti.
Il reward ex-novo conserva il gating che esclude i termini imitativi; la nuova
dipendenza dai dati GRF sinistri vale anche per il teacher diagnostico comune.

## Errore worker

RLlib espone `DictInfoToList(SyncVectorEnv(...))`. Il vecchio callback cercava
gli ambienti direttamente sul wrapper e trattava il vettore come un singolo
ambiente, fallendo prima di PPO. Ora rimuove il wrapper vettoriale, enumera gli
ambienti e rimuove i wrapper individuali. Restano controlli espliciti su
controller, larghezza vettoriale, copertura worker e stato RNG.

Applicato lo stesso fix al payload del builder storico, per non rigenerare il
bug. Quel builder rimane il bundle temporale v1, separato dall'attuale variante
basata sulla fase.

## Verifiche

- 21 test del teacher: stesso angolo a pari fase fra cicli di durata diversa,
  velocità inversamente proporzionale al periodo, derivata numerica coerente,
  shift zero, indipendenza dalla gamba destra e dalla maschera, errori su dati
  mancanti, target/loss/feature coerenti e rifiuto dei checkpoint temporali.
- 16 test worker su variante e payload: veri wrapper Gymnasium, probabilità
  1→0 al reset, contatori, RNG per ambiente, topologie errate e worker mancanti.
- 24 regressioni del bundle preesistente e delle azioni assolute.
- Prova OpenSim sui tre start, con un passo di 10 ms per visibilità 1 e 0:
  stessi target, loss imitative e reward entro 1e-10 a parità di azione/reset;
  teacher clock disponibile e actor clock disabilitato.
- Primo PPO: un aggiornamento, 192 transizioni, 64 per start, tutte visibili,
  actor e critic aggiornati, checkpoint salvato.
- Ripresa completa: secondo aggiornamento, 384 transizioni cumulative,
  distribuzione ancora esatta. Digest di actor e critic prima dell'update
  identici al checkpoint; contatori di ogni worker ripresi da 64 a 128.
  Nessuna iterazione saltata. La ripresa avvia un nuovo episodio fisico,
  conservando lo stato RNG della maschera.
- Parsing dei Python modificati e verifica whitespace sul payload modificato.
  Il controllo globale Git rileva whitespace preesistente in un preset V1
  estraneo a questa modifica; non è stato alterato.

Queste prove attestano il funzionamento operativo, non la qualità del cammino.
La verifica live Windows resta da svolgere con le DLL corrispondenti. Codice e
path usano gli stessi meccanismi portabili del progetto; plugin C++, semantica
SEA, simulatore root e reward weights non sono modificati da questo lavoro.

## File principali

- `phase_target.py`, `env.py`: teacher e integrazione;
- `target_availability.py`: contratto target v2 e fix worker;
- `actor_transition.py`, `training_runtime.py`: versione checkpoint;
- `train_ppo_mlp.py`: descrizione CLI del nuovo target;
- `configs/*.yaml`: provenance del teacher; nuovo smoke a target visibili;
- `tests/conftest.py`, `tests/test_phase_target.py`, `tests/test_target_availability.py`;
- `validation/verify_left_phase_runtime.py` e artefatti collegati sotto;
- `Trajectory Generator/unified_MLP/validation/feasibility39_bundle/target_availability.py`:
  fix del callback nel payload storico.

## Artefatti e avvio successivo

- [Ricevuta finale e hash sorgenti](2026-09-17_left_phase_completion.json).
- [Verifica OpenSim sui tre start](2026-09-17_left_phase_runtime.json).
- [Summary dello smoke con ripresa](../runs/training/left_phase_worker_fix_smoke_2026-09-17/summary.json).
- [Audit del ripristino](../runs/training/left_phase_worker_fix_smoke_2026-09-17/resume_audit.json).
- [Preset per il primo training a visibilità 100%](../configs/pretraining_visible_targets.yaml).

Per il primo confronto usare il preset a target visibili, con curriculum vuoto,
in un nuovo run v2. Il budget lungo e il protocollo statistico di confronto con
V26 restano da definire; lo smoke non avvia la vecchia campagna sospesa.
