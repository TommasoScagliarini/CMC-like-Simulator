# Separazione morphology knee/ankle e critic-only rewarm J20

**Data:** 2026-08-31  
**Stato:** implementazione e verifiche completate  
**Baseline:** actor V26C/J20 derivato dal warm start imitativo V26 di agosto

## Problema

La reward ex-novo utilizzava un solo parametro:

```yaml
morphology_weight: 1.05
```

Il peso moltiplicava la media delle loss morphology di knee e ankle. Non era
quindi possibile aumentare selettivamente la pressione sull'ankle al toe-off
senza modificare contemporaneamente anche il contributo del knee.

In parallelo, la progettazione della nuova rete richiede che il passaggio dalla
reward imitativa alla reward ex-novo avvenga conservando l'actor ma adattando il
critic al nuovo obiettivo. Era quindi necessario verificare che un critic-only
rewarm potesse aggiornare il value function senza alterare actor e
distribuzione esplorativa.

## Soluzione implementata

### Pesi morphology separati

`RewardConfig` espone ora due parametri opzionali:

```yaml
morphology_knee_weight: null
morphology_ankle_weight: null
```

Se un parametro è `null` oppure omesso, eredita il valore storico
`morphology_weight`. La compatibilità con i vecchi YAML è quindi preservata.

La composizione esplicita è:

\[
T_{morphology} =
\frac{w_{knee}L_{knee} + w_{ankle}L_{ankle}}{2}
\]

Quando entrambi i pesi risolti sono uguali al peso storico, il codice usa
deliberatamente la vecchia operazione scalare:

\[
w\frac{L_{knee}+L_{ankle}}{2}
\]

Questo mantiene l'equivalenza esatta della reward, inclusa l'aritmetica
floating-point della baseline.

Sono stati inoltre aggiunti al logging:

- peso knee effettivo;
- peso ankle effettivo;
- contributo morphology knee;
- contributo morphology ankle;
- flag di input non finito separati per i due giunti;
- flag che indica l'attivazione esplicita del contratto split.

Un giunto con peso zero non valuta la propria loss, evitando contaminazioni del
tipo `0 * NaN`. Un peso negativo o non finito causa invece un errore fail-closed.

### Configurazione neutra di controllo

È stata creata la configurazione:

```json
{
  "morphology_weight": 1.05,
  "morphology_knee_weight": 1.05,
  "morphology_ankle_weight": 1.05
}
```

La scelta `1.05/1.05` separa esplicitamente i due canali senza cambiare
l'obiettivo numerico usato nel training precedente. Non rappresenta ancora un
tentativo ankle-specific: il nuovo rapporto dovrà essere definito in un
esperimento successivo.

## Divisione del morphology corridor

La divisione introdotta va distinta in due livelli: **geometria del corridor**
e **peso nella reward**.

### Geometria separata per knee e ankle

Non sono stati creati due file di profilo indipendenti. Il profilo AB06 già
versionato rimane la sorgente unica:

```text
Trajectory Generator/baseline_MLP/morphology_profiles/
ab06_prosthetic_event_warped_mean_std_corridor.json
```

Al suo interno sono però presenti due serie statistiche indipendenti:

```text
coordinates.pros_knee_angle.mean_rad
coordinates.pros_knee_angle.std_rad
coordinates.pros_ankle_angle.mean_rad
coordinates.pros_ankle_angle.std_rad
```

Ciascuna serie contiene `202` punti sulla stessa fase event-warped
`HS -> TO -> HS`, con TO canonico a fase `0.6223299989`. Le statistiche sono
state ricavate da `123` cicli AB06.

Per ogni fase `phi`, il runtime interpola separatamente media e deviazione
standard dei due giunti e costruisce:

\[
C_{knee}(\phi) =
[\mu_k(\phi)-s_k\sigma_k(\phi)-m_k,\;
 \mu_k(\phi)+s_k\sigma_k(\phi)+m_k]
\]

\[
C_{ankle}(\phi) =
[\mu_a(\phi)-s_a\sigma_a(\phi)-m_a,\;
 \mu_a(\phi)+s_a\sigma_a(\phi)+m_a]
\]

Nel critic rewarm i parametri geometrici sono rimasti quelli del training
precedente:

| Parametro | Knee | Ankle |
|---|---:|---:|
| moltiplicatore della deviazione standard | `1.6` | `0.6` |
| margine additivo | `7.5 deg` | `7.5 deg` |
| hard bound inferiore diagnostico | `-1.4 rad` | `-0.6 rad` |
| hard bound superiore diagnostico | `0.0 rad` | `+0.6 rad` |

Gli hard bound morphology sono rimasti diagnostici perché
`morphology_hard_termination_enabled: 0.0`.

Quindi knee e ankle non condividono la stessa banda numerica: condividono
soltanto l'asse di fase e il contratto causale con cui quella fase viene
risolta.

### Loss separata per ciascun corridor

La `served reference` di ogni giunto viene confrontata esclusivamente con la
propria banda. Definendo l'escursione esterna:

\[
d_j = \max(C_{j,min}-q_j,\;q_j-C_{j,max},\;0)
\]

la loss normalizzata è:

\[
L_j = \min\left(25,
\left(\frac{d_j}{C_{j,max}-C_{j,min}}\right)^2\right)
\]

Il runtime conserva separatamente:

- `morphology_knee_loss` e `morphology_ankle_loss`;
- escursione massima knee e ankle in radianti;
- limite inferiore e superiore corrente di ciascun corridor;
- valore della served reference di ciascun giunto;
- escursioni oltre gli hard bound diagnostici.

Nella modalità causale delayed, le loss risolte vengono accumulate
separatamente sul segmento. Entrambe usano lo stesso delay configurato di
`0.04 s`; la divisione non elimina né modifica questa latenza di consegna della
reward.

### Peso separato dopo la divisione geometrica

Prima della modifica, le due loss già esistenti venivano ricombinate subito:

\[
L_{legacy}=\frac{L_{knee}+L_{ankle}}{2}
\]

e ricevevano un solo moltiplicatore. Ora restano separate fino alla composizione
finale:

```text
knee served -> knee corridor -> knee loss -> knee weight --+
                                                         +--> morphology term
ankle served -> ankle corridor -> ankle loss -> ankle weight +
```

Questo consente, per esempio, di lasciare invariata la pressione sul knee e
aumentare soltanto quella sull'ankle. La configurazione `1.05/1.05` usata nel
rewarm mantiene intenzionalmente la ricombinazione storica; un futuro valore
asimmetrico attiverà la nuova possibilità sperimentale.

### Cosa non è stato ancora diviso

Restano condivisi:

- il file di provenance AB06;
- la griglia di fase event-warped;
- il detector e la FSM v3 che risolvono HS/TO;
- il ritardo causale della reward;
- il criterio di disponibilità/fail-closed del segmento.

Non è stato ancora introdotto un corridor ankle cadence-conditioned né una
fascia soft interna. Questi sono sviluppi successivi e non vanno confusi con la
separazione già completata delle bande e dei pesi per giunto.

## Strategia di riadattamento del critic

È stato usato l'actor J20 come sorgente e avviato un run con:

- actor congelato;
- `logstd` congelata;
- critic inizializzato fresco;
- optimizer nuovo;
- una iterazione;
- `4096` step campionati;
- learning rate `1e-4`;
- `10` epoche;
- reward split esplicita `knee=1.05`, `ankle=1.05`;
- FSM v3 e contratto causale J20 invariati.

Actor sorgente:

```text
Trajectory Generator/baseline_MLP/validation/
v26c_july_replica_2026-08-26/j20_runs/
j20_critic_warmup_v26c_2026-08-27_r1/rl_module_last
```

Run valido:

```text
Trajectory Generator/runs/training/
MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31_r2
```

Un primo tentativo nella directory omonima priva del suffisso `_r2` è terminato
prima dell'inizializzazione di Ray per una restrizione macOS su
`psutil/sysctl`. Non ha eseguito update e non costituisce un checkpoint valido.

## Risultati del critic-only rewarm

| Verifica | Risultato |
|---|---:|
| iterazioni completate | `1` |
| step campionati | `4096` |
| actor salvato | byte-identico alla sorgente |
| actor max diff durante il run | `0.0` |
| `logstd` | byte-identica |
| sigma knee | `0.00499999967` |
| sigma ankle | `0.00499999967` |
| mean KL | `0.0` |
| critic digest iniziale | `5ce4ef41d567...` |
| critic digest finale | `b8874489db38...` |
| VF loss | `0.0044071414` |
| VF explained variance | `0.79120505` |

Il critic è quindi cambiato mentre l'actor è rimasto esattamente invariato.

### Salute del rollout di training

| Contatore | Valore |
|---|---:|
| righe osservate | `4096` |
| telemetria mancante | `0` |
| `phase_timeout_stance` | `0` |
| `morphology_causal_contract_failure` | `0` |
| `resync_count_max` | `0` |
| `hs_cancelled_count_max` | `0` |
| disaccordi timeout | `0` |

Tutti i gate dell'audit automatico sono passati.

## Test eseguiti

Sono stati verificati:

1. serializzazione e deserializzazione dei pesi opzionali;
2. equivalenza esatta tra peso globale e split con pesi uguali;
3. composizione con pesi knee/ankle asimmetrici;
4. disattivazione sicura di un giunto con loss non finita;
5. errore fail-closed per loss non finita su un giunto attivo;
6. errore fail-closed per pesi negativi, `NaN` o infiniti;
7. test readiness del morphology corridor già esistenti;
8. immutabilità dei dieci tensori actor;
9. immutabilità delle righe `logstd`;
10. aggiornamento effettivo del critic;
11. salute della FSM e completezza della telemetria.

Il comando aggregato ha completato `14` test con esito `OK`. L'audit del run ha
restituito `ok: true` per tutti i controlli.

Due suite legacy complete contengono assertion su valori della configurazione
attiva già diversi dalle aspettative storiche (`morphology_weight` e timeout
swing). I test mirati della nuova implementazione passano; queste assertion
sono state classificate come drift preesistente delle configurazioni e non come
regressione dello split morphology.

## File modificati o introdotti

### Codice

- `Trajectory Generator/baseline_MLP/reward_function.py`

### Configurazione

- `Trajectory Generator/baseline_MLP/experimental_configs/morphology_joint_split_equal_1p05_2026-08-31.json`

### Test e audit

- `validation/test_morphology_joint_weights.py`
- `validation/audit_morphology_split_critic_rewarm.py`
- `Trajectory Generator/runs/training/MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31_r2/morphology_joint_split_critic_rewarm_audit.json`

### Checkpoint risultante

- `Trajectory Generator/runs/training/MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31_r2/checkpoint_last`
- `Trajectory Generator/runs/training/MLP_V26C_J20_critic_rewarm_morphology_split_equal_1p05_2026-08-31_r2/rl_module_last`

## Interpretazione

Il checkpoint `_r2` non contiene ancora un actor predittivo e non dimostra che
un peso ankle più elevato corregga il toe-off. È un artefatto di controllo che
dimostra tre proprietà:

1. knee e ankle possono essere pesati indipendentemente;
2. la configurazione split neutra non modifica la reward precedente;
3. il critic può essere riadattato senza modificare l'actor J20.

Quando verrà scelto un peso ankle diverso da quello knee, l'obiettivo cambierà
realmente e sarà necessario ripetere il breve critic-only rewarm usando lo
stesso actor di partenza per mantenere controllato il confronto.

## TODO

- Implementare il teacher inverse-governor che produca target raw anticipati e
  coerenti con slew limiter e filtro C2.
- Validare offline uno sweep temporale attorno ai ritardi misurati di `90 ms`
  knee e `100 ms` ankle.
- Eseguire DAgger closed-loop sullo stesso actor 35D usando le nuove label.
- Definire e preregistrare il primo rapporto morphology knee/ankle
  asimmetrico.
- Ripetere il critic-only rewarm quando verrà introdotto il nuovo rapporto.
- Verificare sigma con uno sweep dell'esplorazione effettiva dopo il governor.
- Progettare successivamente il corridor condizionato sulla cadenza.
- Conservare come fase futura la generalizzazione oltre AB06 con dataset EPIC,
  facendo riferimento alla conversazione progettuale precedente.

## Riferimento progettuale principale

La progettazione completa della rete unificata, delle due modalità di reward e
dell'actor predittivo è documentata in:

- `reports/user/2026-08-31_progetto_rete_unificata_imitativa_exnovo_actor_predittivo.md`

I riferimenti storici alla pipeline di giugno/luglio, al warm start V26/J20 e
all'audit di latenza sono raccolti nella sezione finale di quel documento.
