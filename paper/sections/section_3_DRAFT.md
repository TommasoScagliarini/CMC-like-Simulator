# III. Simulation Testbed

[   
### TO BE REVIEWED


    This section describes the physical testbed used to evaluate learned prosthetic
    reference trajectories. The testbed is deliberately modular: the learned policy is
    not allowed to directly actuate muscles, reserves, or motor torques. Instead, it
    will provide a prosthetic kinematic reference, while the validated plant, biological
    tracking loop, static optimization, and low-level prosthesis controller determine
    whether that reference is dynamically feasible. This separation is important for
    the central claim of the paper: the learned trajectory is only meaningful if it can
    be realized by a physical series-elastic prosthesis inside a muscle-driven gait
    simulation.
]

## A. Series-Elastic Prosthesis Model

The prosthetic side is actuated by two OpenSim `SeriesElasticActuator` components,
one at `pros_knee_angle` and one at `pros_ankle_angle`. Each actuator is implemented
as a custom C++ OpenSim component that inherits from `CoordinateActuator`. This
choice keeps the actuator compatible with OpenSim's generalized-force application
and analytic force-gradient machinery, while adding the internal motor states
required to represent a series-elastic actuator (SEA).

[

    Series elastic actuators (SEAs) consist of an electric motor connected in series with a torsional spring. Thus, the joint is not directly driven by the motor, but is actuated through the deflection of the spring. SEAs were selected to actuate the transfemoral prosthesis for two main reasons:

    1. The spring can store and release energy during the gait cycle. This may improve energy efficiency compared with conventional motor actuation and provides intrinsic compliance, allowing the actuator to attenuate impacts during gait.

    2. SEAs are mechanically simpler than variable stiffness actuators.

    Each SEA was implemented as a custom OpenSim plugin. This choice preserves compatibility with the OpenSim software suite.
    ]

The SEA model is described by two physical motor-side state variables:

\[
\theta_m,\quad \omega_m ,
\]

where \(\theta_m\) is the motor-side angle, \(\omega_m\) is the motor angular
velocity, and the joint coordinate angle is denoted by \(\theta_j\). The elastic
torque transmitted through the series spring is

\[
\tau_s = K(\theta_m - \theta_j),
\tag{1}
\]

where \(K\) is the spring stiffness. The motor-side dynamics are

\[
\dot{\theta}_m = \omega_m,\qquad
J_m\dot{\omega}_m = \tau_m - \tau_s - B_m\omega_m .
\tag{2}
\]

Here \(J_m\) is the motor inertia, \(B_m\) is the motor damping, and \(\tau_m\)
is the motor-side torque supplied by the low-level actuator servo described in
Section III-C. The generalized force applied to the OpenSim coordinate is the
physical spring torque,

\[
\tau_{\mathrm{out}} = \tau_s .
\tag{3}
\]

The current AB06 prosthesis model uses the parameter set in Table II. Both
actuators use \(J_m=0.01\;\mathrm{kg\,m^2}\) and
\(B_m=0.1\;\mathrm{N\,m\,s/rad}\). The knee SEA uses
\(K=321\;\mathrm{N\,m/rad}\) with an OpenSim force scale
\(F_{\mathrm{opt}}=100\;\mathrm{N\,m}\). The ankle SEA uses
\(K=500\;\mathrm{N\,m/rad}\) with
\(F_{\mathrm{opt}}=250\;\mathrm{N\,m}\). The servo gains, command limits, and
anti-windup settings are reported with the prosthetic tracking controller.

The SEA dynamics define the primary physical bottleneck for the learned reference.
If the reference trajectory is discontinuous in velocity or acceleration, the
cascade controller can request motor torques that exceed the actuator-servo
bandwidth or drive the SEA into saturation. For this reason, the reference
trajectory generator in Section IV is not connected directly to the actuator
controls. It is connected through a band-limited reference interface and the
cascade controller described below.

**Figure placeholder.** Fig. 2 should show the two-body SEA schematic with
\(\theta_j\), \(\theta_m\), \(K\), \(B_m\), \(J_m\), \(\tau_s\), and
\(\tau_m\).

**Table placeholder.** Table II should report the knee and ankle SEA plant
parameters: \(K\), \(J_m\), \(B_m\), and \(F_{\mathrm{opt}}\).

## B. CMC-Like Muscle-Driven Simulation

The prosthesis is evaluated inside a subject-specific OpenSim simulation based on
subject AB06. The model bundle used for the current experiments is
`AB06_SEASEA_stiff321_500_pi`, with two prosthetic SEA coordinates on the left side
and a muscle-driven biological side. The simulation window is taken from the
experimental treadmill trial and currently spans \(11.99\) to \(21.0\) s. The
integration step is \(1\) ms, and the validated forward-dynamics path uses the C++
SEA plugin with the `rk4_bypass` integration scheme.

The kinematic reference is read from the inverse-kinematics file and preprocessed
before it is used by the controller. The trajectory is resampled on a 1 ms grid and
low-pass filtered with a zero-phase Butterworth filter at 6 Hz. This preprocessing
keeps the spline derivatives usable for acceleration-level tracking and avoids
injecting high-frequency IK noise into the inverse-dynamics and static-optimization
stages.

At each control instant, the CMC-like biological tracking loop computes desired
accelerations for the non-prosthetic coordinates:

\[
\ddot{q}_{\mathrm{des}} =
\ddot{q}_{\mathrm{ref}}
+ K_{p,b}(q_{\mathrm{ref}} - q)
+ K_{d,b}(\dot{q}_{\mathrm{ref}} - \dot{q}).
\tag{4}
\]

The default biological gains are \(K_{p,b}=100\) and \(K_{d,b}=20\), with lower
translation gains on the pelvis coordinates (\(25/10\)) to reduce root-coordinate
drift without forcing the muscle-driven joints through the reserve actuators. The
prosthetic coordinates are excluded from this biological outer loop and from the
biological static-optimization problem; they are handled by the SEA controller.

The generalized force request is computed using a zero-actuator inverse-dynamics
projection. First, all muscle, reserve, and SEA controls are removed. The baseline
acceleration \(\ddot{q}_0\) is computed at the current state with the actuator
contributions zeroed. The acceleration deficit is then projected through the
configuration-dependent mass matrix:

\[
\tau = M(q)\left(\ddot{q}_{\mathrm{des}} - \ddot{q}_0\right).
\tag{5}
\]

The resulting generalized-force vector is split into a biological component and a
prosthetic component:

\[
\tau_{\mathrm{bio}} = \tau[\mathcal{I}_{\mathrm{bio}}],
\qquad
\tau_{\mathrm{pros}} = \tau[\mathcal{I}_{\mathrm{pros}}].
\tag{6}
\]

Only \(\tau_{\mathrm{bio}}\) is passed to the muscle recruitment step. The
prosthetic component is retained as a diagnostic inverse-dynamics oracle, but it is
not used as a feed-forward command in the current prosthetic controller.

Biological recruitment is solved with a muscle-first static-optimization problem.
The optimizer distributes \(\tau_{\mathrm{bio}}\) across muscle activations and
reserve actuator controls:

\[
\begin{aligned}
\min_{a,u_r}\quad
& \sum_i a_i^2 + w_r \sum_j u_{r,j}^2 \\
\mathrm{s.t.}\quad
& A_m(q,\dot{q},l_f)(a-a_{\min})
  + R_r(u_r \odot F_{\mathrm{opt},r})
  = \tau_{\mathrm{bio}}, \\
& a_{\min} \leq a_i \leq a_{\max}, \\
& -u_{r,\max} \leq u_{r,j} \leq u_{r,\max}.
\end{aligned}
\tag{7}
\]

The reserve weight is set to \(w_r=10^6\). Thus, reserves remain available to close
unmodeled or infeasible residual directions, but they are strongly discouraged from
becoming the primary tracking mechanism. In the analysis, reserve usage is treated
as a fidelity gauge rather than as a success metric: low reserve usage indicates
that the simulated motion is supported by the model's muscles and prosthesis,
whereas high reserve usage exposes a dynamic inconsistency, a contact-model
deficit, or an unmodeled actuation demand.

The CMC-like loop therefore provides two pieces of information that are essential
for evaluating learned prosthetic trajectories. First, it determines whether the
biological side can remain muscle-driven while the prosthesis follows the generated
reference. Second, it makes reserve and residual loads available as quantitative
diagnostics of physical validity. This prevents the RL policy from being judged
only by kinematic tracking or reward return.

**Figure placeholder.** Fig. 1 should include the full testbed flow:
IK spline -> biological outer-loop accelerations -> zero-actuator inverse dynamics
-> static optimization -> prosthetic cascade -> SEA plugin -> OpenSim state update.

**Table placeholder.** Table IV should summarize the AB06 model, simulation window,
time step, prosthetic coordinates, biological tracking gains, static-optimization
weights, and GRF mode.

## C. Cascade/PI Prosthetic Reference Tracking

The two SEA coordinates are driven by a high-level prosthetic tracking controller
implemented in Python. This controller receives the prosthetic kinematic reference
\((q_{\mathrm{ref}},\dot{q}_{\mathrm{ref}})\), compares it with the current
prosthetic coordinate state \((q,\dot{q})\), and computes the normalized SEA command
\(u\) sent to the C++ plugin. The current validated mode is a position-to-velocity
cascade followed by an inner velocity PI loop.

For each prosthetic coordinate, the outer proportional position loop computes a
desired cascade velocity:

\[
\dot{q}_{\mathrm{cas}} =
\dot{q}_{\mathrm{ref}}
+ K_{p,o}(q_{\mathrm{ref}} - q).
\tag{8}
\]

The inner loop then converts the velocity tracking error into a torque command:

\[
e_v = \dot{q}_{\mathrm{cas}} - \dot{q},
\tag{9}
\]

\[
\tau_{\mathrm{cmd}} =
K_{p,i}e_v
+ K_{i,i}\int e_v\,dt.
\tag{10}
\]

The integral contribution is clamped per joint to prevent wind-up. Finally, the
controller normalizes the requested torque by the corresponding SEA optimal force:

\[
u = \mathrm{clip}
\left(
\frac{\tau_{\mathrm{cmd}}}{F_{\mathrm{opt}}},
-1,1
\right).
\tag{11}
\]

This \(u\) is the only prosthetic command injected into OpenSim. The C++ plugin
then converts it into a desired spring torque,

\[
\tau_{\mathrm{ref}} = F_{\mathrm{opt}}u,
\tag{12}
\]

and the embedded motor-side servo supplies the \(\tau_m\) that drives the SEA
dynamics in Eq. (2). Thus, the controller requests a spring torque, but the
series-elastic plant determines the actual torque delivered to the coordinate.

The cascade gains used in the current configuration are reported in Table III. The
knee uses \(K_{p,o}=18.85\;\mathrm{s^{-1}}\), \(K_{p,i}=29.2\;\mathrm{N\,m\,s/rad}\),
\(K_{i,i}=1377\;\mathrm{N\,m/rad}\), and a 50 N m inner integral torque limit. The
ankle uses \(K_{p,o}=47.125\;\mathrm{s^{-1}}\), \(K_{p,i}=2.8275\;\mathrm{N\,m\,s/rad}\),
\(K_{i,i}=213\;\mathrm{N\,m/rad}\), and a 200 N m inner integral torque limit.

This controller is deliberately not a learned low-level policy. It is a fixed,
validated tracking interface between the generated kinematic reference and the SEA
plant. Its role is to expose the correct failure mode to the trajectory generator:
when the reference is too aggressive, the controller cannot hide that fact. It
produces larger \(u\), larger motor torque demand, increased spring tracking error,
or saturation. Conversely, a feasible reference remains trackable without changing
the low-level SEA parameters. This is why the reference governor introduced in
Section IV is part of the action interface rather than a post-hoc smoothing
operation.

**Table placeholder.** Table III should report cascade gains, cascade integral
limits, actuator-servo gains, actuator-servo integral and motor-torque limits,
reference-governor velocity/acceleration/jerk limits, and the normalized command
limits.

**Draft note.** Before moving this section to LaTeX, check whether the final paper
uses "series-elastic actuator" or "SEA" consistently, and verify all reported
numerical gains against the final model/config snapshot used for the experiments.
