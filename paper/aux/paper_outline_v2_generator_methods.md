# Paper Outline V2 — Methods as Trajectory Generator Only

**Format:** IEEE conference, double-column, target 8-10 pages.

**Core structural decision:** the `Methods` section describes the trajectory
generation chain: trajectory generator, cascade control, and motor driver. The
simulation apparatus, OpenSim model, SEA plant details, GRF model, reserves, and
validation procedures are part of the experimental apparatus and must move to
`Experiments` and `Results`.

**Revised thesis:** a reinforcement-learning trajectory generator produces knee
and ankle kinematic references, which are passed through a modular cascade
control and motor-driver chain. The generator and its control interface are the
methodological contribution; the simulation testbed is the experimental
apparatus used to evaluate them.

**Current scientific caution:** do not claim final ex-novo trajectory generation
until the baseline ladder, final rollout metrics, and multi-seed evaluation are
complete. The current state supports an architecture/method paper with
diagnostic results; a stronger results section requires successful post-FSM
training.

---

## Title Candidates

1. Plotting the Course: Reinforcement Learning for Kinematic Trajectory Generation in Transfemoral Robotic Prosthesis
2. Learning to Plot the Course: Reinforcement Learning for Kinematic Trajectory Generation in Transfemoral Robotic Prosthesis

---

## Abstract — Blocked on Final Results

Suggested structure:

1. Powered transfemoral prostheses require reference trajectories that are both
   gait-plausible and physically trackable by the actuator.
2. Prior prosthesis RL mainly tunes impedance parameters, target features, or
   actuator commands; here the learned object is the continuous knee-ankle
   kinematic reference.
3. We propose an RL trajectory generator whose action is an absolute prosthetic
   reference passed through a band-limited reference interface.
4. The generator uses deployable actor observations, privileged critic
   information during training, and phase-aware task rewards for the prosthetic
   gait sequence.
5. The generator is evaluated in a muscle-driven OpenSim testbed with two
   series-elastic prosthetic joints.
6. **[BLOCKED]** Report final physical metrics: tracking RMSE, saturation,
   reserve demand, contact/phase success, and multi-seed variability.
7. Takeaway: useful learned prosthetic references must be generated inside an
   action interface that encodes actuator bandwidth and gait-phase feasibility.

---

## I. Introduction


- Motivation: active transfemoral prostheses need knee/ankle references that
  support gait while remaining feasible for compliant actuation.
- Central design choice: learn kinematic reference trajectories instead of
  direct actuator torque commands. Kinematic curves are a simpler and more
  flexible object for a high-level policy: they describe the intended prosthetic
  motion without tying the learned policy to a specific motor torque law,
  actuator bandwidth, or low-level impedance implementation. Torque production
  is then delegated to the fixed prosthetic tracking interface, which can handle
  actuator limits and SEA dynamics.
  **Revision note:** this motivation must be reviewed and sharpened before
  drafting the final Introduction.
- Challenge: a reference generator can easily produce trajectories that are
  kinematically legal but dynamically unusable: out-of-band references, missing
  prosthetic contact events, actuator saturation, and high reserve demand.
- Contributions:
  1. An RL trajectory generator that outputs absolute prosthetic knee and ankle
     reference trajectories.
  2. A band-limited reference interface that converts raw policy actions into a
     physically trackable served reference.
  3. A deployable actor / privileged critic formulation with prosthetic
     phase-event scaffolding for HS-TO-HS gait progression.
  4. An experimental evaluation protocol in a muscle-driven series-elastic
     OpenSim prosthesis testbed, using physical metrics rather than reward as
     the outcome.

---

## II. Related Work

---

## III. Methods — Trajectory Generator

This section should describe the trajectory generation chain as a modular
control architecture. It should include:

```text
Trajectory Generator -> Cascade Control -> Motor Driver
```

The point is to explain how the learned reference is generated and then passed
through the downstream control layers.

### A. Modular Control Chain Context

- Describe the hierarchy in one compact paragraph:
  - the `Trajectory Generator` is the learned high-level module;
  - it outputs knee/ankle kinematic reference trajectories;
  - the `Cascade Control` layer tracks the generated reference and converts
    position-level information into a lower-level command;
  - the `Motor Driver` layer receives the cascade output and produces the final
    actuator-side command used by the downstream plant.
- Keep this subsection descriptive, not derivational.
- State explicitly that the generator is the central object, while cascade
  control and motor driver are included to describe the modular interface from
  generated trajectory to actuator command.
- Forward pointer: plant details, evaluation apparatus, equations, parameters,
  validation, and physical metrics are reported in `Experimental Setup` and
  `Results`.

### B. Problem Formulation

- Define the generator as a policy-driven trajectory pipeline:

```text
observation -> policy -> step/knot command -> C2 filter -> smooth kinematic reference
```

- MDP/POMDP notation:

```text
S, O_actor, O_critic, A, R, gamma
```

- State clearly that the learned object is a kinematic reference, not torque,
  impedance, muscle excitation, or direct actuator command.
- Coordinate scope:
  - `pros_knee_angle`;
  - `pros_ankle_angle`.
- Current action contract:
  - absolute reference knots;
  - `policy_knots = 1`;
  - one command segment per control interval;
  - knee bounds approximately `[-1.5, 0] rad`;
  - ankle bounds approximately `[-0.7, 0.7] rad`.

*TODO*: decide whether the paper should present only the current
single-knot generator or also mention the planned multi-knot extension.

### C. Phase Representation

This subsection should come before the observation and action-generation details
because gait-cycle phase is the cornerstone of ex-novo trajectory generation.
Without an estimate of where the gait cycle is, the policy cannot know whether
it should generate loading, stance support, toe-off, swing flexion, or landing
behavior.

Include:

- define the phase representation used during training;
- explain how phase is reconstructed from phase/event information available to
  the generator;
- explain why phase is necessary for generating a coherent kinematic trajectory
  rather than a sequence of disconnected joint targets;
- describe the runtime phase FSM at a conceptual level:

```text
WAIT_HS
STANCE_AFTER_HS
SWING_AFTER_TO
VALID_CYCLE_COMPLETED
TIMEOUT
INVALID_EVENT
```

- explain the sequence encoded by the FSM:

```text
HS -> stance/loading -> TO -> swing/unloading -> next HS
```

- explain the progress credit only if it remains part of the final reward or
  observation design:

```text
WAIT_HS: 0.00
valid HS: 0.25
valid TO: 0.50
cycle-closing HS: 1.00
```

- briefly state that concrete event detection and validation are described in
  `Experimental Setup`; Methods should focus on the phase representation used by
  the generator.

Key message: phase is not an auxiliary diagnostic; it is the variable that lets
the generator condition the knee/ankle reference on the current point of the gait
cycle.

### D. Observation Design and Actor-Critic Split

Include:

- deployable actor observation philosophy:
  - generated-reference state and history;
  - joint state features needed by the control chain;
  - low-level driver states if available and relevant;
  - phase/contact features if exposed to the actor;
  - no prescribed IK target in ex-novo actor.
- privileged critic:
  - training-only context not used by the actor;
  - prescribed/IK references if used for variance reduction or imitation stages;
  - reward diagnostics.
- Rationale:
  - actor must be deployable by construction;
  - critic may be privileged because it is not used at inference.


### E. Action-To-Reference Generation

This subsection should explain how the policy actually generates a kinematic
trajectory downstream of the phase and observation design.

Include:

- the policy emits step-like/knot-based commands for knee and ankle reference
  generation;
- those steps are not the final smooth trajectory;
- the step sequence is passed through the C2 reference filter/governor to produce
  a continuous, smooth kinematic reference;
- the filter should be described as the mechanism that converts discrete policy
  outputs into a usable trajectory for the cascade controller;
- include the essential C2/jerk-limited model only at the level needed to
  explain trajectory generation;
- explain that this is the ex-novo mechanism: the network generates the
  trajectory itself rather than copying an imitation target.

Do not emphasize:

- absolute-reference target mapping as a separate conceptual point;
- `served reference` versus `target reference` as an imitation-analysis concept;
- reward diagnostics such as out-of-band, command-rate, clipping, or saturation.

Those diagnostics can be defined later in the reward or experimental metrics if
they are needed. In this section, the central message is simply:

```text
policy steps/knots -> C2 filtering -> smooth knee/ankle reference trajectory
```

Do not include actuator/motor equations, cascade gains, or motor-driver
implementation details. Those remain outside the trajectory-generation method.

### F. Reward Design

Keep this subsection brief. The reward should be introduced as the training
objective that supports the generator, not as the primary result and not as a
second full methods section.

Recommended content:

- one compact equation or paragraph defining reward as a weighted sum of terms;
- phase-related terms should point back to Section III-C instead of repeating
  the FSM;
- trajectory regularity terms should point back to Section III-E instead of
  re-explaining the C2 filter;
- contact/phase validity can be mentioned as a small guardrail connected to the
  phase representation;
- morphology corridor should be described only if it is active in the final
  method; otherwise keep it as a diagnostic/future guardrail.

Explicit warning:

```text
Reward is used for optimization; final performance is reported in physical
units, not by return.
```

Training details do not belong in Methods. PPO settings, network size, number of
iterations, seeds, environment runners, and checkpoint/rollout protocol should
be introduced in `Experimental Setup` as the training protocol.

---

## IV. Experimental Setup — Testbed and Protocol

This is where the old "Simulation Testbed" section should move.

### A. Subject and Prosthesis Model

- AB06 single-subject setup.
- Model path/name.
- Prosthetic coordinates.
- SEA knee and ankle installed in the OpenSim model.
- Table: SEA plant parameters and low-level drive/cascade gains.

Keep this concise: the paper needs the testbed to be reproducible, but the
methodological novelty is not the SEA derivation.

### B. Muscle-Driven OpenSim Evaluation Testbed

Briefly:
- CMC-like simulator.
- IK preprocessing:
  - 6 Hz zero-phase Butterworth;
  - 1 ms resampling.
- Biological side:
  - computed-torque tracking;
  - zero-actuator inverse dynamics;
  - muscle-first static optimization;
  - reserves as fidelity gauge.
- Prosthetic side:
  - generator reference -> governor -> cascade/PI -> SEA plugin.
- Hybrid GRF:
  - prescribed/right or sound-side support;
  - online prosthetic GRF for contact/detector;
  - dual profile if used: dynamic support vs event detector.

### C. Training Protocol

This is where PPO and implementation details should be introduced, as the setup
for the learning experiments.

Include:

- PPO/RLlib setup;
- MLP architecture;
- asymmetric actor-critic training setup;
- ex-novo training configuration;
- imitation pretraining only if it is used as an initialization or baseline;
- number of iterations, seeds, environment runners, checkpoint cadence, and
  rollout protocol.

Keep this section factual. It should define how training was run, not introduce
a new methodological block.

### D. Baselines and Ablations 
#### TODO: decide if include this part or not

Define before results:

- B0: IK oracle through the same governor/cascade/SEA.
- B1: raw or weakly governed policy reference.
- B2: generator with governor.
- B3: generator with phase FSM vs timeout-only reward.
- B4 optional: morphology diagnostic/active corridor, if validated.
- Imitation-pretraining baseline vs ex-novo training.

---

## V. Results

Order results so they answer: does the trajectory generator produce feasible
prosthetic references once they are served to the simulated prosthesis?

The figures should be selected from the `Plot` outputs, but not included as an
undifferentiated dump. Each figure should support one explicit result claim.

### A. Generated Prosthetic Trajectories

Show what the policy/generator asks for:

- raw policy command or generated reference knots;
- generated knee/ankle reference;
- served reference after clipping, filtering, or governor;
- served-vs-command RMSE, reported separately as a feasibility diagnostic, not
  as an imitation metric.

Purpose: establish whether the learned generator produces references that are
inside the intended joint/action interface.

### B. Reference Interface and SEA Execution

Show whether the downstream interface and SEA can execute the served reference:

- served reference vs actual prosthetic joint angle;
- tracking RMSE of the served reference;
- SEA torque / spring torque;
- SEA saturation or torque error;
- action clipping and applied-action limits.

Purpose: distinguish a policy/generator problem from a low-level execution
problem.

### C. Contact, GRF, and Gait Events

This is the central result block for ex-novo training:

- left/right GRF;
- online detector events;
- `HS -> TO -> HS` completion;
- contact/load metrics;
- phase FSM state or event counters;
- timeout/termination reason.

Purpose: show whether the generated prosthetic trajectory produces a valid gait
sequence in simulation.

### D. Ex-Novo Training Outcome

If post-FSM training succeeds, report it as a success result:

- gait-cycle success rate;
- valid cycle count;
- timeout rate;
- contact/load metrics;
- knee/ankle generated-reference morphology;
- physical feasibility from Sections A-C.

If training still fails, present this subsection explicitly as diagnostic
failure analysis, not as successful trajectory generation. The current expected
failure-mode framing is:

```text
valid HS -> valid TO -> no new HS -> phase_timeout:swing
```

### E. Reward Diagnostics - To Evaluate

Include only if it materially explains the result or failure mode. These plots
should not be a main Results block by default.

Candidate diagnostics:

- `phase_event_progress_score`;
- `landing_window_contact_score`;
- `phase_timeout_loss`;
- invalid-event counters;
- morphology diagnostics if `morphology_weight = 0.0`.

Possible placements:

- short supporting panel inside Section C or D;
- supplementary figure;
- omitted if the physical plots already explain the outcome.

---

## VI. Discussion

- The key methodological claim is about the generator interface:
  learned references must be constrained by actuator bandwidth and gait-phase
  feasibility.
- The testbed is important because it exposes failures that pure kinematic
  rewards hide: reserve demand, missing prosthetic contact, SEA saturation, and
  torque sign/pathology.
- Discuss why reward can improve while behavior remains invalid.
- Discuss morphology corridor as a guardrail, not imitation.
- Discuss deployability: actor observations vs privileged critic.
- Discuss why the low-level controller is intentionally fixed.

---

## VII. Limitations and Future Work

- Simulation-only.
- Single subject AB06.
- MLP has no memory; phase/contact may require recurrent state or history.
- Hardware transfer remains future work.

---

## VIII. Conclusion

Close on the generator:

```text
This work frames prosthetic RL as kinematic reference generation rather than
low-level control. The central design requirement is that the learned reference
must pass through an action interface that encodes actuator bandwidth, contact
events, and gait-phase feasibility before it is evaluated on a physical
prosthesis model.
```

Keep final performance claims blocked until final results are available.


---

## Open Design Questions For Discussion

1. Should the paper present the FSM as part of the final generator method, or as
   an experimental intervention discovered after timeout-only ex-novo failure?
2. Should morphology corridor appear in the main method if its weight is still
   zero, or should it remain in Discussion/Future Work?
3. Is the main story "architecture toward ex-novo generation" or "successful
   ex-novo generation"? This depends entirely on post-FSM training.
4. Should imitation pretraining be framed as a required stage, an initialization
   option, or a diagnostic baseline?
5. Should the reference governor be described as part of the generator, or as
   the interface between generator and controller? The recommended wording is:
   "generator action interface."
