# Sensibilità della reward LEFT sui trace conservati — 2026-09-24

Analisi offline di 14 rollout / 6.738 righe, senza training o nuove simulazioni.
`analyze.py` ricostruisce il ritorno registrato usando `compute_reward` e la
configurazione salvata in ciascun `rollout_summary.json`, quindi cambia una
famiglia di pesi alla volta. Traiettorie, azioni, contatti e terminazioni restano
quelli registrati: questi risultati **non predicono il comportamento di una
policy riaddestrata**. Output numerico: `metrics.json`, `variants.csv`; provenienza
con SHA-256: `input_hashes.json`.

## Formula verificata

Per questa configurazione:

```
L_actual = knee_position + 2 ankle_position
           + .02 knee_velocity + .04 ankle_velocity
L_served = stessa combinazione, misurata sulla reference filtrata
S = .8 / (1 + 8 L_served) + .2 / (1 + 4 L_actual)
reward = clip(S - P, 0, 1)
         - 2 safety_loss - penetration_loss - .1 ankle_moment_flip_loss
         - 2 out_of_band_loss
```

Le componenti di imitazione sono quadrati degli errori divisi per le escursioni
del rispettivo target, con cap 25. La normalizzazione è presente e corretta;
non è classificata come bug. `P` comprende command rate e termini SEA abilitati.
`reward_terms.penalty` è invece la penalità preliminare dell'ambiente: non deve
essere usata come se fosse `P` della reward effettiva del wrapper.

Nell'imitazione non sono attivi i premi ex novo di avanzamento/completamento
ciclo. Le terminazioni perdono comunque il valore futuro e applicano safety.

## Clipping esterno e saturazioni interne

Il clipping esterno della reward non è una spiegazione convincente del gap sui
checkpoint maturi disponibili: **0/500** righe nei quattro rollout deterministici
LEFT40, LEFT72, oracle LEFT e RIGHT60. Nei dieci rollout rumorosi LEFT72 sono
clippate solo **2/4.738** righe (noise .5 seed123), per un incremento totale della
reward di **0,004996** rispetto al preclip. Abbassare il limite inferiore a −1
non cambia gli altri tredici ritorni.

Esiste invece una saturazione frequente **dentro il costo command rate**:

| Rollout | Costo command rate | Parte segment delta | Knee al cap | Ankle al cap |
|---|---:|---:|---:|---:|
| LEFT40 | 70,864 | 66,636 | 41,6% | 47,0% |
| LEFT72 | 77,831 | 73,280 | 43,4% | 64,0% |
| Oracle LEFT | 20,026 | 19,771 | 5,4% | 5,6% |
| RIGHT60 | 66,501 | 62,711 | 49,4% | 34,0% |

`command_rate_loss` è la media di sei loss: segment delta, velocità, accelerazione,
jerk, intervento dei limiti del governor e variazione `u` SEA. Il suo peso è .05.
La componente dominante, `segment_delta_loss`, confronta **l'endpoint policy con
la reference filtrata all'inizio del segmento**, non due azioni consecutive.
Il cap scatta a 0,25 rad al ginocchio e 0,20 rad alla caviglia (scale .05/.04).
Oltre quel punto questo singolo costo non distingue ulteriori aumenti della
distanza; altri termini della reward possono ancora distinguerli.

Questa semantica può creare un compromesso con l'anticipo necessario a pilotare
il filtro. Tuttavia anche RIGHT60 presenta il fenomeno, mentre il comando diretto
LEFT realizza escursioni più complete con un costo molto minore. Non è dimostrato
che il peso command rate impedisca l'imitazione LEFT; può anche misurare una
conseguenza della policy oscillante. L'oracle conosce il target all'endpoint
10 ms successivo, ulteriore differenza da conservare nell'interpretazione.

## Riscritture dello scalar sui medesimi movimenti

| Variante offline | Delta ritorno LEFT72 | Delta oracle LEFT | Gap oracle − LEFT72 |
|---|---:|---:|---:|
| Reward corrente | 0 | 0 | 165,879 |
| Command rate .05 → .025 | +38,916 | +10,013 | 136,977 |
| Command rate azzerato, solo diagnostica | +77,831 | +20,026 | 108,074 |
| Blend served/actual .8/.2 → .5/.5 | +18,296 | +13,549 | 161,132 |
| Peso posizione ankle 2 → 4 | −49,327 | −32,249 | 182,957 |
| Sharpness served 8 → 4 | +48,571 | +33,335 | 150,643 |
| Peso anti-flip .1 → .2 | −43,108 | 0 | 208,987 |
| Peso penetrazione 1 → 2 | −0,139 | −3,344 | 162,674 |

Il comportamento oracle è preferito dalla reward corrente e da tutte le varianti
esaminate. Ridurre indiscriminatamente command rate o sharpness favorisce di più
la policy osservata peggiore; non costituisce evidenza a favore di quelle modifiche.
Raddoppiare il peso ankle o anti-flip amplia il divario nominale, ma può introdurre
altri compromessi e non dimostra che PPO trovi un percorso migliore.

Le sensibilità locali per 10 bin di fase prescribed sono in `metrics.json`.
Ridurre ipoteticamente del 10% **solo** l'errore di posizione served della caviglia,
con tutto il resto congelato, aumenta il ritorno LEFT72 di 11,429; il massimo
guadagno medio per step è 0,03508 nel bin 30–40% di fase. Per il ginocchio sono
3,517 totali e 0,02467 nel bin 70–80%. Sono sensibilità dello scalar, non azioni
fisicamente realizzate. Le derivate analitiche della reward includono l'indicatore
del clipping (zero fuori dall'intervallo), oltre alla compressione iperbolica.

## Penetrazione: candidato più mirato dei soli picchi di coppia

Confronto tra i dieci step completati prima di ciascun fallimento LEFT72 e gli
stessi indici del rollout deterministico. La riga terminale, eventualmente
parziale, è esclusa. Valori: ritorno finestra del rollout fallito meno nominale.

| Fallimento | Reward corrente | Penetrazione 1 → 2 | Anti-flip .1 → .2 |
|---|---:|---:|---:|
| noise1 seed125, step293–302 | −5,141 | −6,344 | −5,916 |
| noise1 seed127, step456–465 | −0,030 | −1,219 | +0,030 |
| noise.5 seed127, step459–468 | −0,062 | −2,525 | +0,044 |

Nei due casi127 il margine tra finestra pericolosa e nominale è quasi nullo con
la reward corrente. Raddoppiare solo la penalità di penetrazione rende localmente
più netto quel margine; raddoppiare anti-flip non affronta questi casi. È una
motivazione misurata per **un eventuale test controllato** del peso penetrazione,
non una prova di miglioramento dell'apprendimento. Non corregge l'assenza di
segnale prima della soglia20mm e non cambia il dato terminale mancante descritto
sotto. Si tratta di tre episodi, non di una stima generale della robustezza.

## Terminale hard: discontinuità da distinguere dai pesi

Nel noise1 seed127 il passo466 termina `grf_penetration_hard`, conserva il tempo
del passo465 e registra penetrazione/anti-flip pari a zero dopo l'interruzione
del segmento. La loss imitativa **non** è zero: actual .195959 e served .342554.
La reward terminale è −1,900799 = base .102673 − safety2 − out-of-band .003472.

Alla sola frontiera del guard30mm, la formula ordinaria di penetrazione darebbe
già `((.030-.020)/(.028-.020))² = 1.5625`. Applicando quel solo valore allo scalar
terminale congelato, si otterrebbe −3,463299. Non è il recupero di un campione
fisico: momento GRF e dinamica del segmento interrotto non sono disponibili nella
riga e non vengono inventati. Un peso maggiore moltiplica ancora zero se il
termine non è valorizzato. Va quindi verificata la coerenza dello shaping della
terminazione prima di attribuire questo caso alla scelta del peso.

## Discount e verifiche

PPO usa gamma .99 e lambda .9 con step10ms. Il fattore per un costo distante
0,1 / 0,5 / 1 / 1,6 secondi è .904 / .605 / .366 / .200. L'emivita geometrica
dei residui GAE è circa60ms; il bootstrap del value può trasmettere effetti più
lontani, quindi non è un orizzonte rigido. Questi parametri erano anche nel V26:
non sono una differenza di configurazione, ma possono interagire con il diverso
compito. Il ranking per somma episodica non coincide necessariamente con quello
scontato da reset: LEFT40 222,145 vs LEFT72 200,895 non scontati, 59,477 vs60,327
scontati. Questo confronto descrittivo non ricostruisce gli advantage PPO.

Verifiche eseguite: tutte le somme ricostruite concordano con i ritorni salvati
entro1e−8; aggregazione delle sei componenti command rate entro1e−12; ricostruzione
segment delta dalle reference grezze esatta per tutte le righe. Nessun file di
produzione o configurazione attiva modificato, nessun training avviato.
