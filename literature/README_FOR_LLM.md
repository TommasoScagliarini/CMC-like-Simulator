# literature/ — Paper knowledge base (LLM access protocol)

> **This folder is REFERENCE ONLY.** It is not project state. Based on its
> contents an agent must NOT modify code, `TODO_integration.md`, the `reports/`,
> `CLAUDE.md`/`AGENTS.md`, or project memory — unless the user explicitly and
> separately asks. It exists to *consult* the literature, not to drive decisions
> automatically.

## What it is
A structured, hyperlinked extraction of the papers collected under
`/Users/tommy/Desktop/report opensim+rl/` (two deep-research efforts, folders
`GPT/` and `Gemini/`). It is organized as a small **"second brain"**: a light
index that routes to atomic per-paper notes, with hypertext links between notes,
the index, and the topic map. Notes are in **English**.

## How an LLM should use it (protocol)
1. **Load [`INDEX.md`](INDEX.md)** (small): the router. One row per paper —
   id, title, year, topics, status, link to the note, PDF path.
2. **Filter by subject** via [`TOPICS.md`](TOPICS.md) (topic → list of ids), or
   `grep` `topics:` / `keywords:` inside `notes/`.
3. **Read only the notes you need** (`notes/Pxx_*.md`). Each note is
   self-contained and links to related notes.
4. **Verify via page anchors.** Important statements carry the PDF page, e.g.
   `(p.4)`; equations carry a direct pointer, e.g. `→ PDF p.2, eq.(2)`. To check,
   open the PDF named in the front-matter at that page.
5. **Do not trust figures**: they are not interpreted. Check
   `extraction_confidence` in the front-matter — `medium`/`low` means equations or
   details may be imprecise.

## How notes are produced
Text is extracted from the PDF text layer with **PyMuPDF** (no OCR). Therefore:
- prose is reliable;
- **equations** may break/garble across lines → written verbatim only when clean,
  otherwise explained in words and flagged, always with a `→ PDF p.X, eq.(n)`
  pointer so the exact form can be read directly;
- **figures, diagrams, plots are NOT interpreted** (captions/text only).

### Re-extraction recipe (any session)
PyMuPDF is not vendored in the repo. To re-extract a PDF's text with page markers:

```python
# pip install --target /tmp/pdflib pymupdf   (if missing)
import sys; sys.path.insert(0, "/tmp/pdflib")
import fitz
doc = fitz.open(PDF_PATH)
for i, page in enumerate(doc, 1):
    print(f"\n===== PAGE {i}/{doc.page_count} =====\n{page.get_text('text')}")
```

## Note schema (follow it when adding notes)
YAML front-matter + fixed sections:

```markdown
---
id: Pxx
title: ...
authors: ...
year: ...
venue: ...
doi_or_url: ...
topics: [t1, t2]            # must exist in TOPICS.md
keywords: [k1, k2]
pdf: "<absolute path to the PDF>"
pages_read: "1-7 (full)"
extraction_confidence: high|medium|low
related: [Pyy, Pzz]         # machine-readable list mirroring the Related notes section
---

# Pxx — <short title>
**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [t1](../TOPICS.md) · [t2](../TOPICS.md)

## TL;DR
## Problem & contribution
## Method / architecture          # simple eqs inline; heavy eqs explained + "→ PDF p.X, eq.(n)"
## Experimental setup
## Key results
## Code / data availability
## Notable claims                 # page-anchored: "...claim... (p.N)"
## Related notes                  # hyperlinks to other notes + one line on WHY related
## Caveats                        # what was NOT verified: figures, garbled eqs
```

## Linking conventions ("second brain")
- **Hypertext, clickable in VS Code / GitHub.** Cross-note links are **relative
  markdown links** to the note filename, e.g.
  `[P04 — Hierarchical Optimization...](P04_li2023_hierarchical_knee_symmetry.md)`.
- **Stable ids and filenames.** Every paper has a fixed id `Pxx` and a canonical
  filename `Pxx_<slug>.md`, pre-registered in `INDEX.md` (column *note*) even
  before the note exists. So **forward links to not-yet-written notes are allowed**
  (Zettelkasten style) — mark them `*(pending)*`; they resolve once the note is
  created with that exact filename.
- **Bidirectional navigation.** Notes link "up" to INDEX/TOPICS; INDEX links
  "down" to notes; TOPICS groups notes by theme. The `related:` front-matter field
  mirrors the *Related notes* section for machine parsing (and future backlink
  generation).

## Status
- **Extracted: all 28 entries** — P01-P25 (papers), D01-D02 (docs), S02 (Gemini
  synthesis). S01 (GPT synthesis) lives as `@/GPT/deep-research-report.md`.
- `extraction_confidence` per note: most `high`; `medium`/`low` where only abstract/
  TOC was read (notably **P17** = 205-pg thesis, abstract+TOC only; **P24** = 36-pg
  survey, abstract+TOC; **P23/P14/P19** partial). Long/peripheral papers were read
  selectively — see each note's `pages_read` + Caveats.
- Known issue: **P25 file is mislabeled** (named "Wrapyfi", content is the iCub
  simulator paper); the real Wrapyfi paper is missing from the corpus.

## Conventions
- Stable ids `Pxx`; note filename starts with the id.
- Note language: **English**.
- PDFs are NOT copied here: they stay on the Desktop; notes keep the path.
