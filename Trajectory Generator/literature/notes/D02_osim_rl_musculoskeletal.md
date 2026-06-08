---
id: D02
title: "osim-rl — Reinforcement Learning with Musculoskeletal Models (OpenSim-RL documentation)"
authors: "Stanford NMBL (Kidziński et al.)"
year: 2019
venue: "osim-rl docs (OpenSim RL), v1.5 — HTML"
doi_or_url: "github.com/stanfordnmbl/osim-rl ; osim-rl.kidzinski (docs)"
topics: [drl_opensim, osim_rl, env_api]
keywords: [osim-rl, OpenSim, Gym-interface, L2RunEnv, L2M-Walk-Around, AI-for-Prosthetics, conda, reset-step, observation-dictionary]
pdf: "/Users/tommy/Desktop/report opensim+rl/Gemini/paper/1 - Deep Reinforcement Learning per Protesi e OpenSim/Reinforcement learning with musculoskeletal models.html"
pdf_win: "C:\Users\tomma\Desktop\report opensim+rl\Gemini\paper\1 - Deep Reinforcement Learning per Protesi e OpenSim\Reinforcement learning with musculoskeletal models.html"
pages_read: "HTML (quickstart/basic-usage)"
extraction_confidence: high  # official docs; HTML nav stripped
related: [P06, P07, P11]
---

# D02 — osim-rl docs (OpenSim Gym environment)

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [drl_opensim](../TOPICS.md) · [osim_rl / env_api](../TOPICS.md)

## TL;DR
Documentation for **osim-rl**, the OpenSim RL platform behind the Learning-to-Run /
Learn-to-Move competitions ([P06], [P07]). It wraps OpenSim musculoskeletal models as
a **Gym environment** (`reset()`, `step(action) -> obs, reward, done, info`),
cross-platform via Anaconda. This is the **same env pattern the project's own
`CMCLikeProsthesisTrajectoryEnv` follows**.

## Content (quickstart / interface)
- **Install** (cross-platform Win/Linux/Mac 64-bit): `conda create -n opensim-rl -c
  kidzik opensim python=3.6.1`; `conda install -c conda-forge lapack git`;
  `pip install git+https://github.com/stanfordnmbl/osim-rl.git`; verify with
  `python -c "import opensim"`.
- **Gym API**:
  ```python
  from osim.env import L2RunEnv
  env = L2RunEnv(visualize=True)
  obs = env.reset()
  obs, reward, done, info = env.step(env.action_space.sample())
  ```
- **Environments / tracks**: `L2RunEnv` (Learning to Run), **L2M Walk Around** (ML
  & NM tracks), **AI for Prosthetics**; observation **dictionary**, submission/
  evaluation tooling.

## Code / data availability
- **github.com/stanfordnmbl/osim-rl** (open source). Docs sections: Quickstart,
  models, basic/advanced interface, L2M, AI-for-prosthetics, biomechanics, FAQ.

## Notable points
- Canonical **OpenSim → Gym** wrapper (`reset`/`step`/obs/reward/done) — the
  reference for exposing a C++/OpenSim simulator as an RL env (which the project does
  in `osim_trj_cmc_like.py`, with a Gymnasium-like adapter).
- Uses an **observation dictionary** (named state fields) — a design choice relevant
  to the project's observation construction.
- Pinned to older Python (3.6.1) / opensim conda channel — version constraints to
  note for any reuse.

## Related notes
- [P06 — Learning to Run](P06_kidzinski2018_learning_to_run.md), [P07 — Song review](P07_song2021_drl_neuromechanical_locomotion.md) — the competitions/review that use this platform.
- [P11 — DRL Transfemoral (De Vree)](P11_devree_drl_transfemoral.md) — OpenSim DRL on a prosthesis model (same ecosystem).

## Caveats (not verified / limits)
- Documentation page (not a paper); only quickstart/basic-usage read.
- Pinned old versions (Python 3.6.1) — likely dated vs the project's current stack;
  the *API pattern* is the transferable part, not the exact install.
