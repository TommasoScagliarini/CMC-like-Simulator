# 2026-06-30 - Specifica corridor band reward per morphology

## Problema

Il termine morphology gia' integrato nella reward ex-novo usa un corridoio AB06
fase-dipendente come guardrail diagnostico. Prima di attivarlo con peso positivo
serve fissare la semantica corretta: deve essere una **corridor band reward**,
non un tracking verso la media AB06.

Il rischio da evitare e' trasformare il corridoio in imitazione mascherata: se
la reward cresce avvicinandosi alla media, la policy viene spinta a copiare la
traiettoria media invece di restare semplicemente dentro una regione plausibile.

## Disegno formale

Il corridoio va interpretato come bande concentriche:

```text
hard_min        soft_min        inner_min        inner_max        soft_max        hard_max
   |--------------|---------------|================|---------------|--------------|
 terminate/high   mild loss          flat reward       mild loss      high/terminate
```

Semantica:

```text
inner corridor:
  reward = +B
  loss = 0

soft edge band:
  reward = +B - penalty_soft(distance_to_inner)
  oppure reward = +B, loss separata piccola

outside corridor:
  loss forte crescente

hard outside:
  termination oppure loss molto alta post-clip
```

La regola centrale e':

```text
per ogni q dentro [inner_min, inner_max]:
  stesso identico reward
```

Quindi:

- nessun gradiente verso la media AB06;
- nessun incentivo a stare al centro;
- nessuna imitazione mascherata;
- solo incentivo a restare in una regione cinematicamente plausibile.

## Score plateau

Per knee e ankle:

```text
inside_knee  = q_knee  in inner corridor phase-dependent
inside_ankle = q_ankle in inner corridor phase-dependent

morphology_inside_score = mean(inside_knee, inside_ankle)
```

Quindi:

```text
0.0 se entrambi fuori
0.5 se uno dentro e uno fuori
1.0 se entrambi dentro
```

Questo score puo' entrare come bonus plateau:

```text
+ blend_morphology_inside * morphology_inside_score
```

Il bonus non deve dipendere dalla distanza dalla media o dal bordo interno.

## Loss separate

Il bonus plateau va tenuto separato dalle penalita':

```text
morphology_edge_loss
morphology_outside_loss
morphology_hard_violation
```

Interpretazione:

- `morphology_edge_loss`: piccola penalita' nella fascia tra inner e outer;
- `morphology_outside_loss`: penalita' crescente fuori dall'outer corridor;
- `morphology_hard_violation`: violazione anatomica/sicurezza, candidata a
  termination oppure loss molto alta post-clip.

Questa separazione rende leggibile la diagnostica e permette di accendere i
termini in modo graduale.

## Due o tre corridoi

Una parametrizzazione pratica:

```text
inner corridor = mean +/- 1 std + margin interno
outer corridor = mean +/- 1.5 std oppure +/- 2 std
hard corridor  = range anatomico / sicurezza
```

Esempio operativo:

```text
inside inner:
  score = 1
  loss = 0

between inner and outer:
  score = 0.5 oppure 1 con piccola edge_loss

outside outer:
  score = 0
  outside_loss cresce

outside hard:
  terminate
```

Nel primo pass e' preferibile evitare termination morfologiche e usare solo
loss post-clip, per non rendere fragile il training.

## Relazione con FSM e fase

Il corridor band reward richiede una `morphology_phase` affidabile.

Se la fase resta bloccata o sbagliata, il corridoio viene interrogato nel punto
sbagliato del passo e il segnale diventa fuorviante. Per questo l'ordine di
attivazione resta:

```text
1. validare GRF detector + FSM HS -> TO -> HS
2. validare morphology_phase
3. tenere morphology diagnostica con weight 0.0
4. accendere plateau/loss morphology con peso molto basso
5. aumentare solo se la diagnostica resta stabile
```

## Stato rispetto all'implementazione attuale

Implementato oggi:

- profilo AB06 mean/std;
- corridor fase-dipendente;
- loss fuori banda;
- diagnostica morphology;
- `morphology_weight: 0.0` come default diagnostico.

Non ancora implementato:

- `morphology_inside_score` plateau;
- soft/outer/hard corridor separati;
- `morphology_edge_loss`;
- `morphology_outside_loss`;
- `morphology_hard_violation`;
- eventuale termination su hard corridor.

Questa specifica definisce il comportamento desiderato per il prossimo step,
da attivare solo dopo la validazione comportamentale della FSM nel training
ex-novo.
