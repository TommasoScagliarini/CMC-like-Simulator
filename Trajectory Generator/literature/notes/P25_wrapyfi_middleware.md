---
id: P25
title: "[FILENAME SAYS 'Wrapyfi' BUT PDF CONTENT IS:] An Open-Source Simulator for Cognitive Robotics Research: The Prototype of the iCub Humanoid Robot Simulator"
authors: "V. Tikhanoff, A. Cangelosi, P. Fitzpatrick, G. Metta, L. Natale, F. Nori"
year: 2008
venue: "PerMIS'08 (Performance Metrics for Intelligent Systems Workshop)"
doi_or_url: "ACM 978-1-60558-293-1 (PerMIS'08)"
topics: [software_integration, middleware, sim_to_real, env_api]
keywords: [iCub, YARP, ODE-physics, OpenGL-SDL, sim-real-protocol-compatibility, network-wrappers, FILENAME-CONTENT-MISMATCH]
pdf: "/Users/tommy/Desktop/report opensim+rl/Gemini/paper/7 - Architettura Software e Interfacciamento (C++ e Python)/Wrapyfi A Python Wrapper for Integrating Robots, Sensors, and Applications across Multiple Middleware.pdf"
pdf_win: "C:\Users\tomma\Desktop\report opensim+rl\Gemini\paper\7 - Architettura Software e Interfacciamento (C++ e Python)\Wrapyfi A Python Wrapper for Integrating Robots, Sensors, and Applications across Multiple Middleware.pdf"
pages_read: "1-5 of 5 (full)"
extraction_confidence: high  # text clean — BUT the file is mislabeled (see warning)
related: [D01, D02]
---

# P25 — ⚠️ Mislabeled file: PDF is the iCub Simulator paper, not Wrapyfi

**Navigate:** [← INDEX](../INDEX.md) · [TOPICS](../TOPICS.md) · [How to use](../README_FOR_LLM.md)
**Topics:** [software_integration](../TOPICS.md) · [middleware](../TOPICS.md) · [sim_to_real](../TOPICS.md)

> **DATA-INTEGRITY WARNING.** The file is named *"Wrapyfi: A Python Wrapper for
> Integrating Robots, Sensors, and Applications across Multiple Middleware"*, but its
> actual PDF content is a **different, older paper**: the **iCub Humanoid Robot
> Simulator** (Tikhanoff et al., PerMIS'08). The intended Wrapyfi paper (ZeroMQ/YARP/
> ROS middleware bridge for C++↔Python) is **not** in this file. The notes below
> describe what the file *actually contains*. The Wrapyfi content is **missing** from
> the corpus and should be re-sourced if needed.

## TL;DR (of the actual content)
Describes the **open-source iCub humanoid simulator**: ODE physics, OpenGL+SDL
rendering, and crucially the **YARP middleware** so that the **simulator and the real
robot expose the same interface and are interchangeable** (controllable via sockets +
a text protocol). Relevant to the corpus's "C++↔Python software architecture" theme
only via the **middleware / sim-real protocol-compatibility** idea.

## Problem & contribution (actual content)
- Provide a free, open-source, physics-based simulator of the iCub humanoid for
  cognitive-robotics research (RobotCub / ITALK EU projects) (p.1).
- Exact replica of the physical robot (height ~105 cm, ~20.3 kg, **53 DoF**) (p.2).

## Method / architecture (actual content)
- **Physics**: ODE (Open Dynamics Engine), rigid bodies + collision; **Rendering**:
  OpenGL + SDL (cross-platform) for speed (p.2).
- **YARP** (Yet Another Robot Platform) middleware: the simulator uses the **same
  software infrastructure / IPC** as the real robot → "interchangeable from a user
  perspective"; controllable directly via **sockets + text-mode protocol** (YARP not
  required); **network wrappers** allow device remotization across machines (p.2,
  Fig.1). *(This sim-real protocol-compatibility + middleware is the transferable
  idea: decouple the C++ simulator from the Python controller via a messaging layer.)*

## Code / data availability
- Open-source (FP6 RobotCub / FP7 ITALK). iCub simulator + YARP (yarp.it).

## Notable claims (page-anchored)
- Simulator and real robot share the same device API / network interface →
  **protocol-level compatibility** eases transfer of controllers (p.2).
- Middleware/sockets let you integrate "existing controllers in esoteric languages or
  complicated environments" (p.2) — i.e. language-agnostic robot↔controller comms (the
  theme Wrapyfi would also serve).

## Related notes
- [D01 — PyTorch C++ Frontend (LibTorch)](D01_pytorch_cpp_frontend.md) — the other "C++↔Python integration" resource (native C++ inference).
- [D02 — osim-rl docs](D02_osim_rl_musculoskeletal.md) — the OpenSim Gym env interface.

## Caveats (not verified / limits)
- **Filename ≠ content** (the headline issue): this is the iCub simulator paper, not
  Wrapyfi. If the project needs Wrapyfi (ZeroMQ middleware bridge), the actual paper
  must be obtained separately.
- The iCub paper is 2008, humanoid cognitive robotics — only tangentially relevant
  (middleware + sim-real interchange). Figures not interpreted.
