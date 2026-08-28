# V26C — J21: attestazione finale `training_ready`

**Data:** 2026-08-28  
**Stadio:** `V26C_J21_TRAINING_READY_ATTESTATION`  
**Esito:** **TRAINING_READY_ATTESTED — 18/18 gate**  
**Promozione:** `TRAINING_INPUT_ONLY`  
**Training lanciato:** **NO**

## Problema

Il restore audit R3 aveva chiuso correttamente G9, ma il suo contratto lasciava
deliberatamente `training_ready: false`. Era quindi necessario aggregare in un
unico verdetto content-addressed tutte le evidenze già chiuse, senza lanciare un
training o costruire nuovamente il runtime.

La revisione dell'architetto ha inoltre rilevato quattro problemi residui nel
pacchetto preparatorio J21:

1. alcuni testi attivi parlavano ancora di quindici gate, mentre rev1 ne
   definisce diciotto;
2. il marker proposto diceva globalmente che non era avvenuto alcun training,
   negando impropriamente l'unica iterazione critic-only J20 già completata;
3. T15 inferiva l'assenza di una prosecuzione PPO/ex-novo quasi soltanto da
   `promotion` e `next_stage_authorized`;
4. il validatore del GO non rendeva meccanicamente obbligatoria la doppia
   autorizzazione utente + architetto prevista dalla preregistrazione.

## Soluzione e strategia

- Corretto il runner affinché valuti e descriva coerentemente **18 gate**.
- Scopati veridicamente i contatori:
  - J21: **0** iterazioni di training;
  - training actor-updating post-J20: **non avviato**;
  - warm-up critic J20 già svolto: **1** iterazione, 4096 step.
- Rafforzato T15 con evidenza esplicita e pinnata:
  - J19B/J19C: `ppo_updates: 0` e nessuna prosecuzione autorizzata;
  - K1R1: nessun PPO, nessuna chiamata ad `algo.train`, nessun warm-up;
  - warm-up e R3: `ppo_ex_novo: false`;
  - R3 nega esplicitamente PPO, ex-novo, promozione, training readiness e
    qualsiasi stadio successivo.
- Resa vincolante nel validatore l'autorizzazione utente limitata a J21, con
  `training_launch_authorised: false` e con il vincolo verbatim di fermarsi
  prima del training.
- Preservati byte-identici preregistrazioni e DRAFT base/rev1; creato un DRAFT
  rev2 coerente e quindi un GO operativo separato, valido per una sola
  aggregazione e senza retry.
- Eseguito una sola volta l'aggregatore standard-library-only J21.

## Risultato

Foglia:

`Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/j21_runs/j21_training_ready_attestation_v26c_2026-08-28_r1`

Campi conclusivi verificati:

| Campo | Valore |
|---|---:|
| `verdict` | `TRAINING_READY_ATTESTED` |
| `training_ready` | `true` |
| `promotion` | `TRAINING_INPUT_ONLY` |
| gate | `18/18`, nessun fallimento |
| `training_started` | `false` |
| `training_iterations_run` | `0` |
| `j21_execution_training_iterations` | `0` |
| `critic_warmup_iterations_completed` | `1` |
| `child_processes_launched` | `0` |
| `ray_started` | `false` |
| `environment_constructed` | `false` |
| `sampling` / `rollout` | `false` / `false` |
| `launch_authorized` | `false` |
| `ppo_authorized` / `ex_novo_authorized` | `false` / `false` |
| `next_stage_authorized` | `false` |
| `deployable` | `false` |

Il marker `TRAINING_READY_ATTESTED` è presente; `TECHNICAL_INVALID` è assente.
Il result hash coincide con quello registrato dalla receipt e la verifica
post-commit riporta `ok: true`, senza problemi.

## Evidenze e hash principali

| Artefatto | SHA-256 |
|---|---|
| result J21 | `bc5d46522e08777d3de5ec9f24cd058ec282bbde32dc1653857cb45af264d146` |
| receipt J21 | `cf777aedfa5f0beaa4fa26cf2c5b21acc7184d87c9d26566c183ab8f96129477` |
| post-commit verification | `5fea7c9a5eac8e954e7ddfc43d805a1a9cd7103bf8575c598026230a5b702163` |
| marker | `2210aefec837f5cd404fbb4d1fee5b08eb710eaabb0ed88266b257b7ed4ce395` |
| GO operativo | `93566bf642f6856ac1e318a35bc344c2a1fc9e40d04d863f7d5ef1c5c35bc23f` |
| runner | `d33a065c614ad0e75da7828c8831c49c00fd8ea7e3e8745c7acc6ab2f6a873fd` |
| suite | `81609464321e23b8296999dce7a0f9baa3449d4cc4c438a93b1dd868841b0058` |

Ancore storiche preservate:

- prereg base: `df178ffae521aaf46837258ce2036789c6f8ae9f6b4cfb27b4cd11b85d06da79`;
- prereg rev1: `9fa87c120de56d4ef0ae063ce3a4126b94a37ca629fe30710387958d9bcdfdfa`;
- DRAFT base: `b2b4f6014e2fdd43dc9ca0714f24311ee0ba67d3d20ec79488531182cd10716c`;
- DRAFT rev1: `2286c51025508c38fe51735f4830a75c5eca5542fcf0e6bfcc8a78d7e1aca909`.

## Test e verifiche

- Suite ermetica: **381/381** check passati.
- Preflight no-write: **18/18 gate**, **139/139 pin**, destinazione assente.
- Validazione GO: `valid: true`, **141/141 pin**.
- Singola esecuzione J21: exit code 0.
- Post-commit indipendente:
  - set esatto di quattro file;
  - result hash coerente con receipt;
  - commit `ok: true`;
  - marker corretto;
  - nessun processo J21/trainer residuo.

## Diagnostica accettata e limiti del verdetto

- Tutte le nove celle A–I superano la soglia soft diagnostica di 20 mm.
- Solo F e H raggiungono la banda diagnostica luglio di 25 mm, con 4/500
  campioni ciascuna.
- Nessuna cella supera la soglia hard binding di 28 mm.
- La qualifica closed-loop resta svolta a sigma circa 0,005.

`training_ready: true` significa esclusivamente che il checkpoint J20 esistente
è un input verificato per un futuro pilot conservativo. **Non** significa che
il protocollo del pilot, il comando di lancio o gli iperparametri siano già
sigillati o autorizzati.

## File modificati o creati in questa correzione/fase

- `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j21_training_ready_attestation.py`
- `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/test_v26c_j21_training_ready_attestation.py`
- `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j21_training_ready_go_DRAFT_rev2.json`
- `Trajectory Generator/baseline_MLP/validation/v26c_july_replica_2026-08-26/v26c_j21_training_ready_architect_go.json`
- i quattro artefatti della foglia J21 sopra indicata;
- questo report.

## TODO

- [ ] Prima di qualsiasi training: preregistrare e revisionare separatamente il
      pilot PPO conservativo, compresi config, comando, stop rule, monitor e
      rollback; richiedere un nuovo GO e una nuova autorizzazione utente.
- [ ] Valutare esplicitamente sigma e diagnostica di penetrazione nel protocollo
      del pilot; non ereditare automaticamente sigma 0,005.
- [ ] Conservare il TODO di generalizzazione multimodello basato sul dataset
      EPIC e sulla conversazione del 22/08.
- [ ] Conservare LOTO, LOCO, B1R1 e B1R2 come TODO futuri, non come componenti
      della baseline operativa corrente.

**Stop operativo:** raggiunto `training_ready`; nessun training lanciato.
