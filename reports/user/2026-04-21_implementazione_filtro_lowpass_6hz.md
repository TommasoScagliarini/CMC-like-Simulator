# Implementazione dettagliata del filtro low-pass a 6 Hz sulla cinematica

## Scopo del documento

Questo documento descrive in modo operativo e molto dettagliato
l'implementazione del filtro low-pass a 6 Hz applicato alla cinematica di
riferimento del simulatore.

L'obiettivo e' spiegare:

1. quale problema risolve;
2. perche' in un workflow CMC-like ha senso farlo;
3. come e' stato implementato, passo per passo, in modo che un altro agente
   possa riprodurlo senza ambiguita';
4. quali risultati numerici ha prodotto nel simulatore.

Il documento e' scritto assumendo l'architettura "classica" del progetto:

- outer loop di tracking cinematico;
- inner loop SEA sul plugin;
- nessun riferimento ad architetture alternative.

## 1. Il problema

### 1.1 Cosa arriva dal file IK

Il file IK `.sto` contiene una traiettoria di **posizione** per ogni
coordinata del modello:

- coordinate rotazionali in gradi;
- coordinate traslazionali in metri;
- un vettore tempo quasi uniforme, ma non perfettamente pulito.

Questa traiettoria non contiene direttamente:

- velocita' `qdot`;
- accelerazioni `qddot`.

Nel simulatore, pero', servono tutte e tre:

- `q` come riferimento di tracking;
- `qdot` come riferimento di tracking;
- `qddot` come baseline kinematica per la dinamica desiderata e per la parte
  muscle-driven del workflow CMC-like.

Quindi il file IK non viene solo "letto": viene anche **derivato**.

### 1.2 Perche' derivare il dato grezzo e' pericoloso

Ogni derivazione amplifica il contenuto ad alta frequenza.

In pratica:

- la posizione puo' sembrare liscia;
- la velocita' e' piu' rumorosa;
- l'accelerazione puo' diventare molto rumorosa o addirittura patologica.

Questo succede anche se il file IK "visivamente" sembra buono, perche':

1. l'IK contiene inevitabilmente rumore numerico;
2. il tempo non e' perfettamente uniforme;
3. piccole irregolarita' nella posizione diventano molto piu' grandi dopo una
   o due derivate.

### 1.3 Perche' CMC usa un low-pass

In un workflow CMC-like, la cinematica di riferimento non deve essere solo
"corretta" come posizione: deve anche essere **coerente dinamicamente**.

Se si parte da una `q(t)` rumorosa e la si differenzia due volte:

- `qdot_ref` entra nel tracking;
- `qddot_ref` entra nella dinamica desiderata;
- il rumore ad alta frequenza si propaga nella richiesta di coppia;
- il controllore reagisce a componenti che non appartengono alla dinamica del
  gesto ma al rumore del dato.

Il low-pass a 6 Hz serve quindi a fare una cosa molto precisa:

- mantenere il contenuto cinematico lento e fisiologico del passo;
- sopprimere il contenuto spurio ad alta frequenza che esploderebbe nelle
  derivate.

In altri termini: il filtro non serve a "rendere il plot piu' bello", ma a
rendere `qdot_ref` e soprattutto `qddot_ref` usabili dal simulatore.

### 1.4 Il problema specifico trovato nel file usato dal simulatore

Nel file:

```text
data/3DGaitModel2392_Kinematics_q.sto
```

e' stato misurato:

```text
samples_raw        = 7504
samples_unique     = 7503
duplicates_removed = 1
t_start            = 4.23
t_end              = 11.069
dt_min             = 1.3699999996674705e-06 s
dt_max             = 0.0010000000000012221 s
dt_median          = 0.0009999999999994458 s
dt_min_pair        = 6.02599863 -> 6.026
```

Questo significa:

- il file e' quasi uniforme a 1 ms;
- ma non e' perfettamente uniforme;
- contiene 1 timestamp duplicato;
- contiene anche almeno una coppia di campioni quasi coincidenti, separati da
  circa `1.37e-6 s`.

Questa irregolarita' e' sufficiente a rendere fragile l'uso diretto delle
derivate ottenute da una spline costruita sul dato grezzo.

## 2. Strategia scelta

La strategia implementata e' stata:

1. leggere il file IK;
2. rimuovere timestamp duplicati;
3. convertire le rotazioni in radianti;
4. ricampionare il segnale su una griglia **uniforme**;
5. applicare un low-pass Butterworth zero-phase a 6 Hz;
6. costruire le spline cubiche **sul segnale filtrato**;
7. ottenere `q`, `qdot`, `qddot` dalla spline.

La scelta chiave e' questa:

> il filtro non viene applicato dopo la spline; viene applicato prima, sul
> segnale ricampionato uniformemente, e solo dopo si costruisce la spline.

Questo evita di derivare una traiettoria gia' contaminata da contenuto spurio.

## 3. File modificati

L'implementazione e' distribuita in tre punti principali:

- [config.py](/Users/tommy/Documents/CMC-like-Simulator%20-%20Claude/config.py)
- [main.py](/Users/tommy/Documents/CMC-like-Simulator%20-%20Claude/main.py)
- [kinematics_interpolator.py](/Users/tommy/Documents/CMC-like-Simulator%20-%20Claude/kinematics_interpolator.py)

Se il progetto usa anche configurazione YAML, i campi equivalenti sono gia'
presenti in:

- [config.yaml](/Users/tommy/Documents/CMC-like-Simulator%20-%20Claude/config.yaml)

## 4. Parametri introdotti

In `SimulatorConfig` sono stati aggiunti i seguenti campi:

```python
enable_kinematics_lowpass_filter: bool = True
kinematics_lowpass_cutoff_hz: float = 6.0
kinematics_lowpass_order: int = 4
kinematics_resample_dt: float = 0.001
```

Significato:

- `enable_kinematics_lowpass_filter`
  - attiva/disattiva tutto il preprocessamento;
- `kinematics_lowpass_cutoff_hz`
  - frequenza di taglio del low-pass;
- `kinematics_lowpass_order`
  - ordine del Butterworth;
- `kinematics_resample_dt`
  - passo della nuova griglia uniforme prima del filtro.

Valori usati:

```text
cutoff = 6 Hz
order  = 4
dt     = 0.001 s
```

Con `dt = 0.001 s`, la frequenza di campionamento della griglia uniforme e':

```text
fs = 1 / dt = 1000 Hz
```

e quindi:

```text
Nyquist = fs / 2 = 500 Hz
```

La frequenza normalizzata passata al filtro e':

```text
6 / 500 = 0.012
```

## 5. Override da CLI

In `main.py` sono stati aggiunti due flag:

```bash
--disable-kinematics-lowpass
--kinematics-lowpass-cutoff <Hz>
```

Uso tipico:

```bash
python main.py --disable-kinematics-lowpass
```

oppure:

```bash
python main.py --kinematics-lowpass-cutoff 8.0
```

Il comportamento di default resta:

```text
filtro attivo
cutoff = 6 Hz
```

## 6. Implementazione, passo per passo

### 6.1 Parsing del file `.sto`

La funzione privata `_read_sto(filepath)` in `kinematics_interpolator.py`:

1. apre il file;
2. cerca `endheader`;
3. individua la riga delle colonne che inizia con `time`;
4. carica tutte le righe numeriche;
5. separa:
   - `time = arr[:, 0]`
   - `data = arr[:, 1:]`
   - `coord_names = col_names[1:]`

Controlli eseguiti:

- il file deve esistere;
- il numero di colonne deve combaciare con l'header;
- il tempo finale deve coprire la finestra di simulazione;
- il vettore tempo deve essere strettamente monotono dopo la pulizia.

### 6.2 Rimozione dei timestamp duplicati

Subito dopo il parsing:

```python
_, unique_idx = np.unique(time, return_index=True)
if len(unique_idx) < len(time):
    print(...)
    time = time[unique_idx]
    data = data[unique_idx]
```

Effetto:

- se due righe hanno lo stesso timestamp, ne viene tenuta una sola;
- si evita il fallimento della spline;
- si elimina una sorgente banale di non-monotonicita'.

### 6.3 Conversione in radianti

Prima del filtro, per ogni colonna:

```python
deg2rad = np.pi / 180.0
for idx, name in enumerate(coord_names):
    if name not in self._translation_set:
        data[:, idx] = data[:, idx] * deg2rad
```

Quindi:

- le coordinate rotazionali passano a radianti;
- `pelvis_tx`, `pelvis_ty`, `pelvis_tz` restano in metri.

Questo e' importante perche' tutta la pipeline successiva lavora in unita'
fisiche coerenti:

- rad;
- rad/s;
- rad/s^2.

### 6.4 Punto di ingresso del filtro

Nel costruttore di `KinematicsInterpolator`:

```python
if getattr(cfg, "enable_kinematics_lowpass_filter", False):
    time, data = self._lowpass_and_resample(time, data)
```

Quindi il filtro agisce su:

- vettore tempo;
- matrice dati completa di tutte le coordinate.

Solo dopo viene costruita la spline.

### 6.5 Costruzione della griglia uniforme

Dentro `_lowpass_and_resample(time, data)`:

```python
dt = float(getattr(cfg, "kinematics_resample_dt", 0.001))
start = float(time[0])
stop = float(time[-1])
n = int(np.floor((stop - start) / dt)) + 1
uniform_time = start + np.arange(n, dtype=float) * dt
if uniform_time[-1] < stop - 1e-12:
    uniform_time = np.append(uniform_time, stop)
```

Questa parte fa tre cose:

1. parte esattamente dal primo timestamp;
2. costruisce una griglia con passo costante `dt`;
3. se il tempo finale non cade esattamente sul reticolo, aggiunge comunque
   l'ultimo campione `stop`.

Questo dettaglio e' importante: il preprocessamento non deve perdere il
timestamp finale del file IK.

### 6.6 Perche' il ricampionamento viene fatto prima del filtro

Il filtro digitale usato e':

```python
butter(..., output="sos")
sosfiltfilt(...)
```

Questa catena assume un segnale campionato con passo costante.

Se si filtra direttamente un vettore con tempi irregolari:

- il parametro `cutoff` non corrisponde piu' in modo pulito alla frequenza
  fisica voluta;
- il comportamento del filtro non e' ben definito;
- si rischiano risultati distorti nelle derivate.

Per questo l'ordine corretto e':

```text
tempo irregolare -> interpolazione su griglia uniforme -> filtro digitale
```

e non il contrario.

### 6.7 Interpolazione sulla griglia uniforme

Per ogni coordinata:

```python
uniform_data[:, col_idx] = np.interp(
    uniform_time,
    time,
    data[:, col_idx],
)
```

Qui si usa `np.interp`, cioe' interpolazione lineare.

Scelta voluta:

- semplice;
- stabile;
- sufficiente per trasferire il segnale su griglia uniforme;
- il compito di ottenere continuita' di ordine superiore viene demandato alla
  spline costruita dopo il filtro.

### 6.8 Progetto del filtro Butterworth

Dopo il ricampionamento:

```python
fs = 1.0 / dt
nyquist = 0.5 * fs
if cutoff >= nyquist:
    raise ValueError(...)
sos = butter(order, cutoff / nyquist, btype="low", output="sos")
```

Caratteristiche:

- tipo: low-pass;
- famiglia: Butterworth;
- ordine: 4;
- implementazione numerica: SOS, cioe' second-order sections.

Perche' Butterworth:

- risposta in ampiezza monotona;
- nessun ripple in banda passante;
- buona scelta generale quando si vuole attenuare l'alta frequenza senza
  introdurre ondulazioni artificiali.

Perche' SOS:

- maggiore robustezza numerica rispetto ai coefficienti diretti `b, a`,
  soprattutto quando il cutoff e' molto piccolo rispetto a Nyquist.

### 6.9 Applicazione zero-phase

Il filtro viene applicato con:

```python
filtered_data = sosfiltfilt(sos, uniform_data, axis=0)
```

Questo e' un punto cruciale.

`sosfiltfilt` applica il filtro:

1. in avanti;
2. poi all'indietro.

Effetto:

- niente sfasamento netto;
- il contenuto lento non viene ritardato nel tempo;
- la traiettoria filtrata resta allineata temporalmente al riferimento
  originale.

Per un simulatore di tracking, questo e' essenziale: un filtro con ritardo di
fase sposterebbe artificialmente il target nel tempo.

### 6.10 Costruzione della spline finale

Dopo il filtro:

```python
self._splines[name] = CubicSpline(time, col, bc_type="not-a-knot")
```

Qui `time` e `col` sono gia':

- uniformi nel tempo;
- filtrati;
- in unita' corrette.

La spline viene costruita solo a questo punto.

### 6.11 Estrazione di `q`, `qdot`, `qddot`

Nel metodo pubblico `get(t)`:

```python
q[name]     = float(spline(t))
qdot[name]  = float(spline(t, 1))
qddot[name] = float(spline(t, 2))
```

Quindi:

- la posizione e' il valore della spline;
- la velocita' e' la prima derivata analitica della spline;
- l'accelerazione e' la seconda derivata analitica della spline.

Il vantaggio e' che:

- non si differenzia numericamente online;
- la derivata e' liscia;
- il rumore residuo e' quello rimasto dopo il filtro, non quello introdotto da
  differenze finite rumorose.

## 7. Procedura riproducibile per un altro agente

Se un altro agente dovesse reimplementare lo stesso filtro da zero, deve fare
esattamente questa sequenza:

1. Aggiungere in configurazione:

```python
enable_kinematics_lowpass_filter = True
kinematics_lowpass_cutoff_hz = 6.0
kinematics_lowpass_order = 4
kinematics_resample_dt = 0.001
```

2. Aggiungere in CLI:

```bash
--disable-kinematics-lowpass
--kinematics-lowpass-cutoff
```

3. Nel parser del file IK:
   - leggere `time`, `coord_names`, `data`;
   - eliminare timestamp duplicati;
   - validare monotonicita' stretta.

4. Convertire in radianti tutte le coordinate non traslazionali.

5. Se il filtro e' attivo:
   - costruire `uniform_time`;
   - interpolare ogni colonna con `np.interp`;
   - progettare il Butterworth con:

```python
sos = butter(order, cutoff / (0.5 * fs), btype="low", output="sos")
```

   - applicare:

```python
filtered_data = sosfiltfilt(sos, uniform_data, axis=0)
```

6. Costruire `CubicSpline` sui dati filtrati.

7. Usare la spline per restituire posizione, velocita' e accelerazione.

8. Loggare chiaramente che il preprocessamento e' attivo, per esempio:

```text
[KinInterp] Ready ... (low-pass 6 Hz, dt=0.001 s)
```

## 8. Risultati ottenuti

### 8.1 Finestra breve `4.26 -> 5.30 s`

Metriche sulle accelerazioni derivate `qddot`, confrontando filtro spento e
acceso.

#### Senza filtro

```text
pros_knee_angle   RMS 33.817   max 64.673   rad/s^2
pros_ankle_angle  RMS 7.251    max 18.834   rad/s^2
ankle_angle_r     RMS 29.188   max 75.724   rad/s^2
mtp_angle_r       RMS 2.888    max 20.430   rad/s^2
```

#### Con filtro a 6 Hz

```text
pros_knee_angle   RMS 33.294   max 65.356   rad/s^2
pros_ankle_angle  RMS 6.774    max 14.016   rad/s^2
ankle_angle_r     RMS 26.511   max 67.026   rad/s^2
mtp_angle_r       RMS 2.054    max 4.625    rad/s^2
```

L'effetto sulla finestra breve si vede soprattutto su:

- `pros_ankle_angle`;
- `ankle_angle_r`;
- `mtp_angle_r`.

Il knee prostetico, su quella finestra specifica, non mostrava ancora il caso
peggiore del dato grezzo.

### 8.2 Finestra lunga `4.26 -> 11.06 s`

Qui emerge il motivo vero per cui il filtro era necessario.

#### Senza filtro

```text
pros_knee_angle   RMS 32.538   max 475.805  rad/s^2
pros_ankle_angle  RMS 7.225    max 77.382   rad/s^2
ankle_angle_r     RMS 31.258   max 319.109  rad/s^2
mtp_angle_r       RMS 3.167    max 27.984   rad/s^2
```

#### Con filtro a 6 Hz

```text
pros_knee_angle   RMS 31.521   max 67.812   rad/s^2
pros_ankle_angle  RMS 6.743    max 15.706   rad/s^2
ankle_angle_r     RMS 28.875   max 97.961   rad/s^2
mtp_angle_r       RMS 2.206    max 6.848    rad/s^2
```

### 8.3 Riduzione dei picchi massimi su finestra lunga

Riduzione percentuale del picco `max |qddot|`:

```text
pros_knee_angle   -85.7%
pros_ankle_angle  -79.7%
ankle_angle_r     -69.3%
mtp_angle_r       -75.5%
```

Questo e' il risultato piu' importante del lavoro.

Non stiamo parlando di un piccolo affinamento:

- il knee prostetico passa da circa `476 rad/s^2` a `68 rad/s^2`;
- l'ankle prostetico da circa `77` a `16 rad/s^2`;
- l'MTP da circa `28` a `6.8 rad/s^2`.

### 8.4 Interpretazione pratica dei risultati

Dopo il filtraggio:

- la cinematica di riferimento resta sostanzialmente la stessa a bassa
  frequenza;
- le derivate diventano molto piu' fisiche;
- i picchi spurii di accelerazione si riducono drasticamente;
- il contenuto ad alta frequenza che entrava nella richiesta dinamica viene
  abbattuto.

In pratica, il simulatore smette di inseguire accelerazioni "inventate" dal
rumore numerico del file IK.

## 9. Cosa non fa il filtro

E' importante essere precisi anche su questo.

Il filtro a 6 Hz:

- non corregge errori di modello;
- non sostituisce il tuning dei gain;
- non risolve da solo problemi di tracking;
- non elimina tutte le fonti di contenuto ad alta frequenza nella pipeline.

Quello che fa e':

- pulire il riferimento cinematico prima di derivarlo;
- evitare che la parte di controllo e dinamica lavori su `qdot_ref` e
  `qddot_ref` patologici.

## 10. Conclusione

Il low-pass a 6 Hz e' stato introdotto per rendere la cinematica di
riferimento coerente con un workflow CMC-like.

La sequenza corretta implementata e':

```text
parse IK
-> rimuovi duplicati
-> converti in rad
-> ricampiona a 1 ms
-> low-pass Butterworth zero-phase 6 Hz
-> spline cubica
-> q / qdot / qddot
```

Questa scelta e' stata necessaria per due ragioni:

1. il file IK non ha un vettore tempo perfettamente regolare;
2. le accelerazioni derivate dal dato grezzo avevano picchi non fisici molto
   alti.

Il risultato finale e' un riferimento cinematico molto piu' pulito, con
riduzioni dei picchi di `qddot` fino all'ordine del 70-85% sulle coordinate
osservate.

Se un altro agente deve riprodurre il comportamento del filtro, deve copiare
non solo il valore "6 Hz", ma l'intera pipeline:

- pulizia tempo;
- ricampionamento uniforme;
- Butterworth in SOS;
- applicazione zero-phase;
- spline costruita solo dopo il filtro.
