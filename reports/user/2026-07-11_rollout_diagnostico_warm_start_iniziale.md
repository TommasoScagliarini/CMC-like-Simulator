# Rollout diagnostico del warm-start iniziale

Data: 2026-07-11

Instruction check token: CMC_AGENT_OK_2026

## Obiettivo

Valutare il comportamento deterministico di
`rl_module_initial_warm_start` prima di qualsiasi aggiornamento PPO, usando la
configurazione ex-novo congelata con soglie di penetrazione `15/25 mm`.

Il porting tensoriale della rete era gia' validato. Questo test verifica invece
se il comportamento trasferito sia un punto di partenza utile per H1.

## Problema

La correttezza tensoriale del transplant non garantiva che la policy imitativa
fosse compatibile con il dominio ex-novo. Restavano da verificare sopravvivenza,
HS/TO/cicli, penetrazione, clipping e dinamica SEA sotto gait clock disabilitato,
FSM corrente e slew limiter target.

## Rollout principale: configurazione target 15/25 mm

Artefatto:

```text
validation/warm_start_rollout_runs/2026-07-11_initial_deterministic_full/
```

Risultato:

```text
step eseguiti / richiesti       = 35 / 500
durata simulata                 = 0.35 s
episode return                  = -2.148138
terminated / truncated          = true / false
causa                           = grf_penetration
penetrazione massima            = 26.1064 mm
primo superamento 15 mm         = step 27, 16.1831 mm
step oltre 15 / 22 / 25 mm      = 9 / 4 / 1
HS / TO / cicli validi          = 1 / 0 / 0
stance elapsed al terminale     = 0.350 s
```

La terminazione non e' dovuta a timeout FSM, range articolare o saturazione
degli input SEA. Il carico finale e' `0.607 BW`; la penetrazione cresce in modo
quasi monotono fino al guard.

Diagnostica actor e attuazione:

```text
|action raw| max                = 1.7813
step con clipping               = 6 / 35
valori action clippati          = 6 / 70 = 8.57%
step interessati dal slew       = 35 / 35
mean slew-limited fraction      = 0.9714
reserve norm mean / max         = 229.11 / 311.99 Nm
SEA input saturation max        = 0
knee excursion                  = 0.0884 rad
contact-support clawback finale = 1.4819
```

Il clipping e' concentrato sulla caviglia agli step `1-5` e `7`. La
penetrazione continua anche dopo che le action tornano nell'intervallo, quindi
il clipping iniziale non e' da solo la causa del fallimento.

## Controfattuale hard guard 30 mm

Artefatto:

```text
validation/warm_start_rollout_runs/2026-07-11_initial_deterministic_hard30_counterfactual/
```

Con gli stessi pesi e dinamica, alzando soltanto il guard a `30 mm`:

```text
step                            = 39
penetrazione terminale          = 30.7159 mm
HS / TO / cicli validi          = 1 / 0 / 0
```

Il margine aggiuntivo produce solo quattro step e nessun TO. Il fallimento a
`25 mm` non e' quindi un artefatto marginale della soglia.

## Controfattuale senza slew limiter

Artefatto:

```text
validation/warm_start_rollout_runs/2026-07-11_initial_deterministic_no_slew_counterfactual/
```

Il source imitativo era stato addestrato senza slew limiter. Disabilitandolo
nel target, ma mantenendo il guard a `25 mm`:

```text
step                            = 195
episode return                  = -4.881303
HS / TO / cicli validi          = 2 / 1 / 1
primo TO valido                 = step 52
primo ciclo valido              = step 94
periodo ciclo                   = 0.881 s
penetrazione terminale          = 25.0950 mm
|action raw| max                = 2.3508
step con clipping               = 24 / 195
reserve norm mean / max         = 377.92 / 850.98 Nm
knee motor accel max            = 6320.63 rad/s^2
knee range min                  = -1.30085 rad
SEA input saturation max        = 0
```

Il ciclo esiste, ma e' troppo rapido rispetto al limite nominale minimo di
`0.9 s`, non riceve il bonus contact-support perche' il TO e' troppo precoce e
usa reserve e accelerazioni molto elevate. La seconda stance termina comunque
per penetrazione.

## Interpretazione

Il porting della rete resta corretto. Il problema e' il trasferimento
comportamentale tra due contratti diversi:

- il source usava il gait clock sonoro, neutralizzato nel target;
- le nuove feature FSM partono con peso nullo;
- il source era stato addestrato senza slew limiter;
- la strategia source richiede variazioni rapide dei target che il limiter
  target filtra completamente;
- rimuovere il limiter recupera eventi ciclici, ma espone una politica troppo
  aggressiva e residual-driven.

Non emerge quindi una semplice scelta corretta fra limiter acceso o spento:
con limiter la policy non scarica il piede prima della penetrazione; senza
limiter produce un ciclo non sicuro e non conforme alla configurazione target.

## Soluzione e strategia raccomandata

H1 warm-start diretto puo' essere eseguito tecnicamente, ma non parte da un
vantaggio comportamentale rispetto alla fresh policy: il rollout deterministico
ricade nello stesso failure mode pre-TO. Eseguirlo ora avrebbe soprattutto
valore esplorativo e rischierebbe di attribuire al PPO un problema gia'
presente allo step zero.

La strategia raccomandata e' una breve ri-adattazione imitativa nel dominio
target, mantenendo:

- osservazioni ex-novo `39/84` e gait clock disabilitato;
- detector/FSM correnti;
- slew limiter `2.5/2.0 rad/s`;
- start e soglie `15/25 mm` congelati.

L'obiettivo e' insegnare all'actor comandi compatibili con il limiter e rendere
utili le feature FSM, conservando il feature extractor trasferito. Solo dopo un
rollout deterministico con almeno un TO/ciclo valido e dinamica accettabile
conviene avviare H1 PPO.

## Test e verifiche eseguiti

- rollout deterministico principale con configurazione congelata `15/25 mm`;
- controfattuale con hard guard a `30 mm`;
- controfattuale senza slew limiter e con hard guard ripristinato a `25 mm`;
- analisi delle trace per eventi FSM, penetrazione, clipping, reserve, range
  articolari, coppie e accelerazioni SEA;
- verifica che nessun test abbia modificato pesi, YAML o codice di training.

## File modificati

Nessun file di codice o configurazione e' stato modificato. Sono stati creati
soltanto gli artefatti dei tre rollout e questo report.

## TODO

- [x] Eseguire il rollout deterministico completo del warm-start iniziale.
- [x] Verificare che il guard a `25 mm` non sia l'unica causa con un
      controfattuale a `30 mm`.
- [x] Isolare il domain shift del slew limiter.
- [x] Definire ed eseguire la ri-adattazione imitativa sotto il contratto
      target congelato. Risultati in
      `2026-07-11_riadattamento_imitativo_target_domain.md`.
- [x] Ripetere il rollout deterministico: il round DAgger 2 raggiunge un ciclo
      valido e return positivo. Il full episode e il gate sul picco reserve
      restano aperti per H1/H2.
- [ ] Eseguire H1/H2 warm-start solo dopo il nuovo gate comportamentale.

## Rettifica successiva

Il risultato iniziale a 35 step era contaminato da un'incoerenza del target
slew limiter: il clamp target-to-target era ancorato all'uscita filtrata del
governor invece che all'ultimo target accettato. Dopo la correzione, il source
ported arriva a 396 step e un ciclo, ma con return negativo, clipping, eventi
invalidi e reserve persistenti molto elevati. Le conclusioni e il candidato
aggiornato sono documentati in
`2026-07-11_riadattamento_imitativo_target_domain.md`.
