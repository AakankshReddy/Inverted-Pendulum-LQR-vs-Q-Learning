# Inverted Pendulum: LQR vs Q-Learning Comparison & Stress-Test Suite

A full comparison of a **white-box, model-based controller (LQR)** against
a **black-box, model-free controller (tabular Q-learning)** for stabilizing
a torque-driven inverted pendulum — in both plain MATLAB and Simulink —
across nominal conditions, single disturbances, combined disturbances, and
plant/model mismatch.

---

## 1. System Model

Single torque-driven pendulum, `theta = 0` defined as the **upright
(unstable) equilibrium**.

Nonlinear equation of motion (the ground-truth plant used for *all*
simulation, regardless of which controller is being tested):

```
I * theta_ddot = m*g*l*sin(theta) - b*theta_dot + u + d
```

- `u` = control torque (from LQR or RL), saturated at `+/- u_max`
- `d` = disturbance torque (impulse / constant / sinusoidal / noise, or
  any combination summed together)

### Physical parameters (`pendulum_params.m`)
| Symbol | Meaning | Value |
|---|---|---|
| `m` | pendulum mass | 0.2 kg |
| `l` | length to center of mass | 0.3 m |
| `I` | moment of inertia about pivot | 0.006 kg*m^2 |
| `b` | viscous friction coefficient | 0.01 N*m*s/rad |
| `g` | gravity | 9.81 m/s^2 |
| `u_max` | actuator torque saturation | 1.5 N*m |
| `slew_rate` | max \|du/dt\| (actuator bandwidth limit) | 80 N*m/s |
| `dt` | simulation timestep | 0.01 s |

The **slew-rate limit is applied identically to both controllers** inside
`simulate_controller.m` — it exists to fairly model a real actuator's
finite bandwidth, not to advantage one controller. It matters much more
for the RL agent's discrete, bang-bang-style actions than for LQR's
naturally smooth output.

---

## 2. White-box controller: LQR (`design_lqr.m`)

Linearizes the plant about `theta=0`:
```
A = [0, 1; m*g*l/I, -b/I]
B = [0; 1/I]
```
Solves the continuous-time Riccati equation via `lqr(A,B,Q,R)` with
`Q = diag([50, 1])`, `R = 4`.

**Final verified gain:** `K = [4.1728, 0.5379]`, control law `u = -K*x`.

**Closed-loop eigenvalues:** `[-7.0929, -84.2206]` — both stable (real,
negative). Independently re-derived and confirmed against the simulation
output during this project (not just assumed).

### Why `R=4` and not something smaller
An earlier version used `R=0.5`, giving a much more aggressive gain
(`K=[10.6059, 1.4486]`) with a fast pole at `-236 rad/s`. At the
simulation's `dt=0.01s`, explicit Euler integration is only stable for a
pole if `|1 + lambda*dt| < 1`; for `lambda=-236` this gives `|-1.36|>1`,
i.e. **the integrator itself becomes numerically unstable for that mode**,
producing dense high-frequency control chatter that looked like a
controller problem but was actually a numerical-resolution problem.
Raising `R` to 4 softened the gain enough (`-84.22 rad/s` pole) to be
comfortably resolved at this timestep — this is documented here because
it's a legitimate, explainable methodology decision, not an arbitrary
tweak.

---

## 3. Black-box controller: Q-learning (`train_qlearning.m`)

Tabular Q-learning; the agent never sees `m, l, I, b, g` — only
`(state, action, reward, next_state)` tuples from interacting with the
plant.

### State/action discretization
- **Non-uniform angle bins**: dense near `theta=0` (~0.375 deg
  resolution within +/-15 deg of upright) so the agent can resolve small
  errors, coarser further out.
- **Non-uniform angular-velocity bins**: dense near zero, coarser at the
  extremes.
- **Non-uniform torque actions**: fine steps (0.025 N*m) near zero torque
  plus the two saturation extremes, so the agent isn't forced to
  bang-bang between max torques just to hold position.

### Reward shaping
Raw "+1 if upright" is too sparse to learn from by random exploration, so
the reward is:
```
r = -(theta^2) - 0.05*(theta_dot^2) - 0.001*(u^2)   [+1.0 bonus inside a tight band near upright]
```
This follows the standard potential-based reward-shaping argument (Ng,
Harada & Russell, 1999) — dense enough to give a usable gradient from
anywhere in state space.

### Training hyperparameters
- 12,000 episodes, 4s each (400 steps at dt=0.01)
- Learning rate: decays from 0.3 to 0.02
- Exploration (epsilon-greedy): decays from 1.0 to 0.02
- Discount factor: 0.98
- RNG pinned (`rng(7,'twister')`) **inside** `train_qlearning.m` itself,
  so training is reproducible regardless of what random calls happened
  earlier in a script — this was a real bug found and fixed mid-project
  (results were silently varying run-to-run before this fix).

### Deployment-time dead zone
Once `|theta| < 1.5 deg` and `|theta_dot| < 0.08 rad/s`, the policy is
forced to the nearest-to-zero available torque action instead of
whatever the table's argmax says. This suppresses a classic tabular-Q
artifact — chattering between two adjacent bins right at a discretization
boundary — without giving the agent any privileged model information
(LQR's `u -> 0` at `x=0` too; this just prevents RL's *deployed* policy
from oscillating across a grid edge it can't resolve).

---

## 4. Simulation harness (`simulate_controller.m`)

Runs **either controller on the true nonlinear plant** — LQR's linear
model is only used to *derive* `K`, never to simulate the response. This
is what makes the comparison fair.

### Disturbance model
`disturbance` can be:
- `[]` — none
- a single struct with a `.type` field: `'impulse'`, `'constant'`,
  `'sinusoidal'`, or `'noise'`
- a **cell array of structs** — all channels are summed simultaneously,
  enabling combined multi-type stress tests (see `stress_test.m`,
  Section 6 below)

| Type | Required fields | Behavior |
|---|---|---|
| `impulse` | `.t`, `.magnitude`, `.duration` (default 1 timestep) | fires once, for `duration` seconds starting at `.t` |
| `constant` | `.t`, `.magnitude` | turns on at `.t` and never turns off |
| `sinusoidal` | `.t`, `.magnitude`, `.freq` | `magnitude * sin(2*pi*freq*(t-t0))`, starting at `.t` |
| `noise` | `.t`, `.magnitude` | `magnitude * randn()` every step, starting at `.t` |

### Metrics computed per run (`compute_metrics`)
- `steady_state_error` — mean `|theta|` over the last 20% of the run
- `settling_time` — first time `|theta|` stays inside `band` permanently
- `max_abs_theta` — peak deviation over the whole run
- `control_effort_rms` — RMS of the applied torque
- (if a disturbance was applied) `peak_deviation_post_disturbance`,
  `recovery_time` (measured from the **peak**, not from `t=disturbance.t`
  — see bug note below), `final_theta_after_disturbance`

### Bugs found and fixed during this project (kept here for the record)
1. **Recovery time falsely reported as ~0.** The original code searched
   for the first in-band sample starting at `hit_idx`, but `theta` at
   that exact index is the state *recorded before* the disturbance was
   applied that step — trivially near zero — so it always looked like
   instant recovery. Fixed by searching only *after* the peak deviation.
2. **Settling time falsely reported as ~T (near the full sim duration).**
   Caused by setting the settling `band` (0.02 rad) *tighter* than a
   controller's own steady-state error — it could only "settle" via a
   lucky noise dip right at the end of the window. Fixed by exposing
   `band` as a parameter and using 3 deg (0.0524 rad), comfortably above
   both controllers' real error floors.
3. **RL peak angle blew out to ~112 deg after adding the actuator
   slew-rate limit.** An initial `slew_rate=15 N*m/s` throttled RL's
   bang-bang actions far more than LQR's smooth output, because the
   plant's natural instability growth rate (`sqrt(m*g*l/I) ~ 9.9 rad/s`)
   was faster than the actuator could ramp to full torque. Fixed by
   raising `slew_rate` to 80 N*m/s — verified by checking LQR's peak
   angle stayed ~unaffected across the change (it should be, given its
   smooth control law), confirming the fix targeted the right cause.

---

## 5. Files

### Core plant + controllers
| File | Purpose |
|---|---|
| `pendulum_params.m` | Physical constants, actuator limits, slew rate, dt |
| `pendulum_dynamics.m` | Nonlinear equations of motion (ground-truth plant) |
| `design_lqr.m` | Linearizes, solves Riccati equation, prints K and closed-loop eigenvalues |
| `train_qlearning.m` | Tabular Q-learning, reward shaping, non-uniform discretization |
| `simulate_controller.m` | Runs either controller on the nonlinear plant; saturation, slew-rate limiting, single/combined disturbance injection, metrics |

### Comparison and stress testing
| File | Purpose |
|---|---|
| `main_compare.m` | Master script: design LQR, train RL, nominal + single-disturbance comparison, metrics table, animation |
| `stress_test.m` | 5 harsh scenarios (impulse, constant bias, sinusoidal, noise, parameter mismatch) + 10s combined multi-disturbance scenario with animation |

### Animation
| File | Purpose |
|---|---|
| `animate_pendulum.m` | Single-pendulum animation from a theta(t) trajectory |
| `animate_pendulum_compare.m` | Basic side-by-side LQR vs RL animation, optional GIF export |
| `animate_pendulum_pro.m` | Polished version: motion trail, live torque bar, dark theme — used by `stress_test.m` |

### Simulink
| File | Purpose |
|---|---|
| `build_qr_pendulum_simulink.m` | Programmatically builds `.slx` matching the block-diagram structure (integrators, gravity/friction/LQR gain blocks, saturation), plus a 4-channel disturbance summing junction (Pulse Generator, Constant, Sine Wave, Band-Limited White Noise) |
| `init_pendulum_workspace.m` | Sets all plant/controller/disturbance variables in the base workspace; wired as the model's `PreLoadFcn` and `InitFcn` so it self-loads on open or on every sim start |

### Report
| File | Purpose |
|---|---|
| `REPORT.md` | Methodology, white-box vs black-box trade-off table, practical limitations, resume-bullet framing |

---

## 6. How to run

```matlab
main_compare.m       % design LQR, train RL (~1-2 min for 12,000 episodes), nominal + single disturbance comparison, prints metrics table, animates result
stress_test.m         % 5 harsh scenarios + 10s combined-disturbance scenario, prints table, plots, animates
```
Requires the Control System Toolbox (`lqr()`).

For Simulink:
```matlab
build_qr_pendulum_simulink.m   % run once -> generates QR_Pendulum_Stress.slx
```
Open the `.slx` — parameters auto-load. To change a disturbance
(impulse magnitude, sine frequency, noise power, etc.), edit the
relevant variable in `init_pendulum_workspace.m` and re-run it (or
re-open the model) before hitting Run.

**Simulink gotcha worth knowing:** `PreLoadFcn` only fires when a model
is opened fresh — not when it's already open in the same session (e.g.
right after `build_qr_pendulum_simulink.m` creates it). If parameters
look missing, just run `init_pendulum_workspace()` manually once.

**Simulink Band-Limited White Noise gotcha:** the `Seed` parameter must
be a bare numeric expression (`noise_seed`), not string-wrapped
(`num2str(noise_seed)`) — the latter throws a mask-evaluation error.

---

## 7. Verified results (current tuning)

**LQR**: `K = [4.1728, 0.5379]`, `Q=diag([50,1])`, `R=4`. Closed-loop
eigenvalues `[-7.09, -84.22]` — independently re-derived via
`lqr(A,B,Q,R)` in a separate check and confirmed to match exactly.

### Nominal (20 deg initial offset, no disturbance)
| Metric | LQR | Q-learning |
|---|---|---|
| Settling time (s) | 0.28 | 0.12 |
| Steady-state error (rad) | 0.000 | ~0.025 (discretization floor) |
| Peak \|theta\| (rad) | 0.349 | 0.349 |
| Control effort RMS (N*m) | 0.057-0.07 | 0.10-0.11 |

### Under 0.6 N*m impulse disturbance
| Metric | LQR | Q-learning |
|---|---|---|
| Peak deviation post-hit (rad) | 0.010-0.019 | 0.035-0.040 |
| Recovery time (s) | 0.02-0.04 | 0.07 |
| Final theta post-disturbance (rad) | 0.000 | ~0.01-0.03 |

---

## 8. Key findings for write-up / interview

1. **LQR reaches exact zero steady-state error**; Q-learning has a
   nonzero floor set by state/action discretization — a direct,
   quantifiable cost of going model-free with a tabular method. This
   floor was reduced from ~2 deg to ~1.4 deg over the course of tuning
   by making the grid finer near equilibrium, but never eliminated —
   that residual is the honest, reportable limit of the approach.
2. **LQR uses more control effort** for the same task in most tunings
   tried — the optimal analytical gain is more aggressive than what a
   discretized RL policy naturally expresses, though this is sensitive
   to the specific `Q`/`R` chosen (a softer LQR tuning narrows this gap).
3. **Neither controller has integral action**, so a *constant* bias
   disturbance produces a permanent nonzero offset for LQR (see
   `stress_test.m`, "Constant bias" scenario) rather than being fully
   rejected — a real, reportable limitation, not a bug.
4. **Large sustained disturbances near actuator saturation can push
   both controllers into continuous rotation** rather than
   stabilization (observed directly in the constant-bias stress test
   animation) — the true robustness ceiling in that regime is the
   actuator's torque limit, not the control law itself.
5. **Parameter mismatch** (mass/friction changed after the controllers
   were designed/trained, without retuning) tests true robustness
   rather than just disturbance rejection — this is the scenario where
   LQR's model-dependency is most directly exposed, since its gain is
   fixed at design time from the nominal plant.
6. **Numerical integration resolution interacts with controller
   aggressiveness.** A fast closed-loop pole (large `|lambda|`) can be
   numerically unstable under explicit Euler at a fixed `dt` even though
   the true continuous-time system is stable — worth knowing before
   attributing chatter to "the controller" rather than "the simulator."

---

## 9. Known simplifications
- Single-link pendulum (not cart-pole) — a 2-state system keeps tabular
  Q-learning tractable within a reasonable training budget.
- Q-learning is tabular, not deep RL — doesn't scale to higher
  dimensions, but keeps training fast (~minutes, not hours) and the
  comparison interpretable (you can inspect the full Q-table).
- LQR's gain and RL's training both assume the nominal plant parameters
  in `pendulum_params.m`; the parameter-mismatch stress test
  intentionally violates this to test robustness, not baseline
  performance.
- The disturbance model injects torque directly into the plant equation
  — it does not model sensor noise, actuator faults, or communication
  delay, which would be natural next steps for a more complete
  robustness study.
# Adaptive-Noise-Cancellation-Investigation
# Adaptive-Noise-Cancellation-Investigation
