# V26C J20 — Il precedente di luglio della sonda di gradiente

**Data**: 2026-08-27
**Tipo**: indagine documentale di sola lettura, chiusa
**Esecutore**: Opus 5, effort xhigh — braccio operativo. Codex architetto e gate owner.

## Domanda

Prima di K1R1: si era già presentato, nella pipeline di giugno/luglio, un problema analogo a
quello attuale — un test di backprop del critic con gradiente nullo, in particolare sul primo
layer, dovuto a un batch di verifica degenere, oppure confuso con un critic realmente non
addestrabile? E quali correzioni furono allora adottate?

## VERDETTO

**Lo stesso pattern di stimolo degenere esisteva già a luglio, in produzione, dentro una sonda
autograd. Ma non produsse alcun fallimento equivalente, perché il suo perimetro era diverso e
il suo gate più debole. E il critic, a luglio, fu provato addestrabile per altra via — su dati
reali, mai con i gradienti.**

Quindi: **non «nessuna evidenza», e non «stesso problema già osservato»**. La radice
metodologica è ereditata; il fallimento no.

---

## 1. Correzione esplicita di una mia conclusione precedente

Nella prima risposta a questa indagine ho affermato che **«prima di agosto in questo progetto
non è mai esistita una sonda di gradiente»**. **È falso.**

L'errore è mio e ha una causa precisa: ho chiuso la ricerca di `.backward(` con `head -20` su
**86 occorrenze**, e il file decisivo non rientrava nelle prime venti. Ho quindi presentato
come negativo esaustivo il risultato di una lista troncata. Le due ricerche documentali che
avevo lanciato in parallelo lo hanno trovato; l'ho poi verificato riga per riga di persona
prima di correggere.

Restano valide, e cambiano soltanto di ruolo, le altre constatazioni: giugno e luglio ebbero
episodi **reali** di critic debole o fresco, non artefatti di misura, e non sono quindi il
precedente cercato. Il precedente cercato è nel **codice**, non nella prosa.

---

## 2. Il precedente esatto

**`validation/verify_h1_readiness.py`**, funzione `_gradient_audit`, introdotta dal commit
**`bdbf99c1`** del **2026-07-14** (`git log --follow` restituisce quel solo commit; mtime del
file `Jul 12 13:28`).

| riga | contenuto |
|---|---|
| 29 | `def _gradient_audit(module_path: Path) -> dict[str, Any]:` |
| 39 | `output_layer = module.pi[-1]` |
| 42 | `{Columns.OBS: torch.zeros((4, n_full), dtype=torch.float32)}` |
| 44 | `logits.sum().backward()` |
| 45 | `weight_gradient = output_layer.weight.grad` |
| 46 | `bias_gradient = output_layer.bias.grad` |
| 49-54 | `mean_gradient_abs_max` = massimo su **weight e bias** aggregati, righe `[:action_dim]` |
| 55-60 | `logstd_gradient_abs_max` = idem sulle righe `[action_dim:]` |
| 62 | `"pass": mean_gradient_abs_max > 0.0 and logstd_gradient_abs_max == 0.0,` |

È **lo stesso idioma di K1**: batch di zeri → `backward()` → ispezione di `.grad` → verdetto.
Scritto sei settimane prima.

Ed è anche l'origine di una convenzione che agosto ha ereditato: alla riga 62 **un gradiente
nullo è il criterio di PASS** (per il log-std congelato). K1 ha applicato quella convenzione a
un tensore dove lo zero significa tutt'altro.

## 3. L'artefatto che lo registra

**`validation/warm_start_port_runs/2026-07-12_sigma0003_h1_readiness/h1_readiness.json`**,
righe 34-44:

```json
"gradient_audit": {
  "pass": true,
  "freeze_logstd": true,
  "mean_gradient_abs_max": 4.0,
  "logstd_gradient_abs_max": 0.0,
  "logstd_output_weight_abs_max": 0.0,
  "logstd_output_bias": [-5.80914306640625, -5.80914306640625]
}
```

## 4. L'algebra, e perché il PASS di luglio non provava sensibilità

Per uno strato lineare `y = Wx + b`:

- **peso**: `∂L/∂W = (∂L/∂y)·xᵀ`. Con `x = 0` è **identicamente nullo**, qualunque cosa faccia
  il freeze. È il difetto di K1.
- **bias**: `∂L/∂b = ∂L/∂y`, **indipendente da `x`**. Per `logits.sum().backward()` su un batch
  di 4 righe, `∂(Σ logits)/∂b_j = 4`.

Il valore registrato è **esattamente `4.0`**, cioè la dimensione del batch. E poiché
l'ingresso dell'ultimo layer è una `tanh`, con `|h| ≤ 1` si ha `|∂L/∂W| = |Σ_righe h| ≤ 4`:
**il massimo fu raggiunto dal bias**. L'asserzione `> 0.0` della riga 62 fu dunque soddisfatta
da una quantità che **non dipende dallo stimolo**.

Conseguenza netta: **la sonda di luglio era già cieca sul ramo di sensibilità.** Il suo
verdetto fu corretto, ma non perché il test discriminasse — perché il caso era favorevole. Due
accidenti la protessero, non due precauzioni:

1. guardava `module.pi[-1]`, l'**ultimo** layer, il cui ingresso è `tanh(b) ≠ 0` anche con
   `x = 0`. Un layer più indietro, su `pi_encoder[0]`, avrebbe visto lo stesso zero di agosto;
2. il ramo `> 0.0` era garantito dal bias.

Ciò che la sonda **discriminava davvero** era il ramo `== 0.0`: con `freeze_actor=True`
`_detach_actor_gradient` restituisce `logits.detach()`, `.grad` è `None` e la riga 48 dà
`{"pass": False, "reason": "actor output gradients are missing"}`.

## 5. Perimetro: luglio non poteva manifestare il FAIL di K1

`_policy_logits` — `Trajectory Generator/baseline_MLP/asymmetric_rl_module.py:152-159` —
calcola `self.pi(self._actor_in(batch))` e passa per i soli detach. La riga 40 di
`verify_h1_readiness.py` azzera i gradienti di `module.pi`.

**`vf` e `vf_encoder` non entrano mai nel grafo.** La sonda di luglio verificava il freeze del
log-std sull'**attore**; non interrogava il critic, e quindi **non poteva** far emergere
`P24_frozen_module_still_gives_the_critic_gradient`, il check fallito da K1 su
`vf_encoder.0.weight`.

Nota collaterale, verificata: `validation/test_asymmetric_rl_module.py`, la cui docstring
recita *«Focused gradient tests for the asymmetric Gaussian actor»*, **non istanzia mai il
modulo** (`grep -c "AsymmetricActorCriticTorchRLModule("` → **0**): importa le due funzioni
libere di detach e le esercita su un tensore costante `[[1.0, 2.0, -3.0, -4.0]]`. Fra il
15/06, creazione del modulo, e il 01/08 nessun test manda un'osservazione attraverso il
modulo, in nessuna direzione.

## 6. Come luglio provò davvero che il critic era addestrabile

Su **dati reali**, mai con i gradienti.

`validation/critic_warmup/2026-07-13_deployable_sigma0005_iter1/robustness_critic_warmup_gate.json`,
righe 82-97:

| riga | campo | valore |
|---|---|---|
| 85 | `sampled_steps` | **4096** step reali campionati dall'ambiente |
| 89 | `initial_critic_digest` | `89dbf05c…c236052` |
| 90 | `warmed_critic_digest` | `c0b159c9…7b27fdb` |
| 91 | `restored_critic_digest` | `c0b159c9…7b27fdb` — **identico al caldo** |
| 92 | `vf_loss` | `0.3440060914` |
| 93 | `vf_explained_variance` | `0.5983695984` |
| 94 | `mean_kl` | `0.0` |
| 95-96 | `actor_digest_before` / `after` | `5616be85…83dd2bf7`, **identici** |
| 97 | `actor_max_abs_parameter_change` | `0.0` |

E la lettura contemporanea, `reports/user/2026-07-13_actor_robust_deployable_critic_warmup.md:167-175`:

```
actor prima/dopo     5616be85...2bf7, identico
critic fresco        89dbf05c...6052
critic caldo         c0b159c9...7fdb
critic dopo restore  c0b159c9...7fdb
```
> «Il `policy_loss=-0.0313` è una metrica calcolata dalla loss PPO, ma **non produce gradienti
> sull'actor congelato**. `KL=0` e il digest bit-identico confermano che non è avvenuto alcun
> update della policy.»

È la **stessa domanda di K1**, posta a luglio e risolta senza toccare autograd: digest attore
invariante, `KL = 0`, digest critic che cambia e si riproduce dopo restore, `vf_loss` ed EV
finiti come diagnostici non vincolanti.

## 7. La distinzione

| | luglio | K1 (agosto) |
|---|---|---|
| **radice comune** | stimolo zero, gate cieco sul ramo di sensibilità | stimolo zero, gate cieco sul ramo di sensibilità |
| perimetro | solo attore, `module.pi[-1]` | critic, `vf_encoder.0.weight` |
| ramo discriminante | `== 0.0` (freeze log-std) — reale | `> 0.0` (gradiente presente) — degenere |
| ramo cieco | `> 0.0`, garantito dal bias | lo stesso, ma su un peso di **primo** layer |
| esito | PASS corretto, per caso favorevole | FAIL_CLOSED, falso allarme |
| il critic fu provato addestrabile? | **sì**, su 4096 step reali, per digest/EV/restore | non era l'oggetto della sonda |

**Stessa radice di harness. Nessun fallimento documentato del critic reale a luglio.**

## 8. Implicazione

- **K1 è un `FAIL_CLOSED` del nuovo gate, non una prova che il critic non sia addestrabile.**
  Il tensore mancante lo era per algebra dello stimolo, non per una proprietà del modulo.
- **K1R1 con stimolo non degenere è la correzione necessaria**, e chiude anche la lacuna che
  luglio aveva lasciata aperta: richiede norma **finita e strettamente positiva** su tutti e
  sei i tensori critic, non la semplice presenza.
- **Ma il gate conclusivo resta quello di luglio**: warm-up reale su dati campionati più
  restore audit. Un gradiente prova che il grafo è connesso; il digest fresco → caldo →
  ripristinato prova che il critic si è mosso e che il movimento è recuperabile. È già il
  gate `G9` della preregistrazione J20, ed è più forte di qualunque sonda sintetica.

## 9. Stato

- **K1 immutabile**: leaf `FAIL_CLOSED` 28/29, GO, runner, preregistrazione e report invariati.
- **K1R1**: **NO-GO, non eseguito.** Nessun GO K1R1, nessun leaf, nessuna esecuzione.
- Questa indagine non ha modificato codice, preregistrazioni, artefatti o GO.

---

## Fonti, con righe verificate

| percorso | righe | cosa |
|---|---|---|
| `validation/verify_h1_readiness.py` | 29, 39, 42, 44, 45-46, 49-60, 62 | la sonda autograd di luglio: batch di zeri, backward, solo `pi[-1]`, PASS `> 0.0` |
| `validation/warm_start_port_runs/2026-07-12_sigma0003_h1_readiness/h1_readiness.json` | 34-44 | `mean_gradient_abs_max: 4.0`, `logstd_gradient_abs_max: 0.0`, `pass: true` |
| `Trajectory Generator/baseline_MLP/asymmetric_rl_module.py` | 152-159 | `_policy_logits`: solo `self.pi`, mai `vf`/`vf_encoder` |
| `validation/test_asymmetric_rl_module.py` | 1, 17, 22-30 | «focused gradient tests» che non istanziano mai il modulo |
| `validation/critic_warmup/2026-07-13_deployable_sigma0005_iter1/robustness_critic_warmup_gate.json` | 82-97 | warm-up reale: 4096 step, digest, `vf_loss`, EV, `mean_kl`, attore invariato |
| `reports/user/2026-07-13_actor_robust_deployable_critic_warmup.md` | 167-175 | lettura contemporanea: `KL=0` e digest bit-identico al posto dei gradienti |
| `reports/user/2026-06-11_monitoraggio_critico_vf_metrics_prossimo_training.md` | 4-5, 11-17, 31-33 | giugno: EV cronicamente negativa — critic **realmente** debole |
| `reports/user/2026-06-10_observation_space_realistico_critico_privilegiato.md` | 22-23, 66-69 | la correzione di giugno: critic privilegiato asimmetrico |
| `reports/user/2026-07-12_h1_single_iteration_h2_rejected.md` | 177-179 | luglio: critic «intenzionalmente fresco» |
| git | `bdbf99c1`, 2026-07-14 | unico commit che introduce `verify_h1_readiness.py` |

## STOP

Indagine chiusa. Nessun file di codice, preregistrazione, artefatto o GO modificato. Nessuna
esecuzione. **K1R1 resta NO-GO.**
